#include <Eigen/Geometry>
#include <robotic_arm_mtc/pick_place_task.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RObotic_arm_4dof_ws.pick_place_task");

namespace {
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
    return Eigen::Translation3d(values[0], values[1], values[2]) *
           Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
    return tf2::toMsg(vectorToEigen(values));
};
} 

namespace robotic_arm_mtc {

    void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                    const moveit_msgs::msg::CollisionObject& object) {
        if (!psi.applyCollisionObject(object))
            throw std::runtime_error("Failed to spawn object: " + object.id);
    }

    moveit_msgs::msg::CollisionObject createTable(const pick_place_task_parameters::Params& params) {
        geometry_msgs::msg::Pose pose = vectorToPose(params.table_pose);
        moveit_msgs::msg::CollisionObject object;
        object.id = params.table_name;
        object.header.frame_id = params.table_reference_frame;
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        object.primitives[0].dimensions = { params.table_dimensions.at(0), params.table_dimensions.at(1),
                                            params.table_dimensions.at(2) };
        pose.position.z -= 0.5 * params.table_dimensions[2];  // align surface with world
        object.primitive_poses.push_back(pose);
        return object;
    }

    moveit_msgs::msg::CollisionObject createObject(const pick_place_task_parameters::Params& params) {
        geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
        moveit_msgs::msg::CollisionObject object;
        object.id = params.object_name;
        object.header.frame_id = params.object_reference_frame;
        object.primitives.resize(1);
        object.primitives[0].type =
        shape_msgs::msg::SolidPrimitive::BOX;
        object.primitives[0].dimensions =
        {
            params.object_dimensions.at(0),
            params.object_dimensions.at(1),
            params.object_dimensions.at(2)
        };
        pose.position.z += 0.5 * params.object_dimensions[0];
        object.primitive_poses.push_back(pose);
        return object;
    }

    void setupDemoScene(const pick_place_task_parameters::Params& params) {
        rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
        moveit::planning_interface::PlanningSceneInterface psi;
        if (params.spawn_table)
            spawnObject(psi, createTable(params));
        spawnObject(psi, createObject(params));
    }

    bool isNamedState(const moveit::core::RobotState& current,
                    const moveit::core::RobotModelConstPtr& model,
                    const std::string& group_name,
                    const std::string& state_name,
                    double tolerance = 0.02)
    {
        moveit::core::RobotState target(model);
        const auto* jmg = model->getJointModelGroup(group_name);
        target.setToDefaultValues(jmg, state_name);

        std::vector<double> current_vals;
        std::vector<double> target_vals;

        current.copyJointGroupPositions(jmg, current_vals);
        target.copyJointGroupPositions(jmg, target_vals);

        for (size_t i = 0; i < current_vals.size(); ++i)
        {
            if (std::abs(current_vals[i] - target_vals[i]) > tolerance)
                return false;
        }
        return true;
    }

    PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

    bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node, const pick_place_task_parameters::Params& params) {
        
        RCLCPP_INFO(LOGGER, "Initializing task pipeline");

        task_.reset();
        task_.reset(new moveit::task_constructor::Task());

        Task& t = *task_;
        t.stages()->setName(task_name_);
        t.loadRobotModel(node);

        auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

        auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);

        // Set task properties
        t.setProperty("group", params.arm_group_name);
        t.setProperty("eef", params.eef_name);
        t.setProperty("hand", params.hand_group_name);
        t.setProperty("hand_grasping_frame", params.hand_frame);
        t.setProperty("ik_frame", params.hand_frame);

        // --------------------------------------------------------------------------------------
        // FIXED: Added baseline starting state generator inside the predicate check block
        // --------------------------------------------------------------------------------------
        {
            auto current_state = std::make_unique<stages::CurrentState>("current state");

            auto applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
            applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
                if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                    comment = "object with id '" + object + "' is already attached and cannot be picked";
                    return false;
                }
                return true;
            });
            t.add(std::move(applicability_filter));
        }

        // --------------------------------------------------------------------------------------
        // FIXED: Combined Fallback Container with continuous Property Inheritance (group)
        // --------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------
        // 2. Fallbacks Container for Initialization
        //--------------------------------------------------------------------------------------
        {
            auto init_fallback = std::make_unique<Fallbacks>("initialize fallback");
            
            // FIX: Declare and immediately SET the property so parents can resolve it instantly
            init_fallback->properties().declare<std::string>("group");
            init_fallback->properties().set("group", params.arm_group_name);

            // =========================================================================
            // STRATEGY 1: Directly try moving to Initialize 
            // =========================================================================
            {
                auto direct_init = std::make_unique<stages::MoveTo>("direct move initialize", sampling_planner);
                direct_init->properties().configureInitFrom(Stage::PARENT, { "group" }); // Inherits perfectly now
                direct_init->setGoal(params.arm_initialize_pose);
                direct_init->properties().set("timeout", 0.5); 
                
                init_fallback->add(std::move(direct_init));
            }

            // =========================================================================
            // STRATEGY 2: Move to Home position FIRST, then to Initialize
            // =========================================================================
            {
                auto home_sequence = std::make_unique<SerialContainer>("home recovery sequence");
                
                // FIX: Declare and SET the property on the serial sequence wrapper container
                home_sequence->properties().declare<std::string>("group");
                home_sequence->properties().set("group", params.arm_group_name);

                // Step A: Move to Home
                auto step_home = std::make_unique<stages::MoveTo>("fallback move home", sampling_planner);
                step_home->properties().configureInitFrom(Stage::PARENT, { "group" });
                step_home->setGoal(params.arm_home_pose);
                home_sequence->insert(std::move(step_home));

                // Step B: From Home, move to Initialize
                auto step_init = std::make_unique<stages::MoveTo>("fallback move initialize", sampling_planner);
                step_init->properties().configureInitFrom(Stage::PARENT, { "group" });
                step_init->setGoal(params.arm_initialize_pose);
                home_sequence->insert(std::move(step_init));

                init_fallback->add(std::move(home_sequence));
            }

            // Add the fallback container to the main task pipeline
            t.add(std::move(init_fallback));
        }
        
        //--------------------------------------------------------------------------------------
        // open hand stage to define the start state for picking
        //--------------------------------------------------------------------------------------
        Stage* initial_state_ptr = nullptr;
        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_open_pose);
            initial_state_ptr = stage.get();  
            t.add(std::move(stage));
        }

        /****************************************************
         * Move to Pick                                     *
         ***************************************************/
        {
            stages::Connect::GroupPlannerVector planners = { { params.arm_group_name, sampling_planner }};
            auto stage = std::make_unique<stages::Connect>("move to pick", planners);
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            t.add(std::move(stage));
        }

        /****************************************************
         * Pick Object                                      *
         ***************************************************/
        Stage* pick_stage_ptr = nullptr;
        {
            auto grasp = std::make_unique<SerialContainer>("pick object");
            t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
            grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

            {
                auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
                stage->properties().configureInitFrom(Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose(params.hand_open_pose);
                stage->setObject(params.object_name);  
                // stage->setAngleDelta(2*M_PI );
                stage->setAngleDelta(M_PI_2);
                stage->setMonitoredStage(initial_state_ptr);  

                RCLCPP_INFO(LOGGER, "Grasp frame: %.3f %.3f %.3f",
                    params.grasp_frame_transform[0], params.grasp_frame_transform[1], params.grasp_frame_transform[2]);

                auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(100);
                wrapper->setMinSolutionDistance(0.0);
                wrapper->properties().set("timeout", 0.5);
                wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
                
                RCLCPP_INFO(LOGGER, "group: %s", t.properties().get<std::string>("group").c_str());
                wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(Stage::INTERFACE,{"target_pose"});

                grasp->insert(std::move(wrapper));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
                stage->allowCollisions(params.object_name,
                    t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(), true);
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
                stage->setGroup(params.hand_group_name);
                stage->setGoal(params.hand_close_pose);
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
                stage->attachObject(params.object_name, params.hand_frame);  
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
                stage->allowCollisions({ params.object_name }, { params.surface_link }, true);
                grasp->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
                stage->allowCollisions({ params.object_name }, { params.surface_link }, false);
                grasp->insert(std::move(stage));
            }

            pick_stage_ptr = grasp.get();  
            t.add(std::move(grasp));
        }

        /******************************************************
         * Move to Place                                      *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::Connect>(
                "move to place", stages::Connect::GroupPlannerVector{ { params.arm_group_name, sampling_planner } });
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            t.add(std::move(stage));
        }

        /******************************************************
         * Place Object                                       *
         *****************************************************/
        {
            auto place = std::make_unique<SerialContainer>("place object");
            t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
            place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

            {
                auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject(params.object_name);

                geometry_msgs::msg::PoseStamped p;
                p.header.frame_id = params.object_reference_frame;
                p.pose = vectorToPose(params.place_pose);
                p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
                stage->setPose(p);
                stage->setMonitoredStage(pick_stage_ptr);  

                auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(2);
                wrapper->properties().set("timeout", 0.5); 
                wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
                wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
                place->insert(std::move(wrapper));
            }

            {
                auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
                stage->setGroup(params.hand_group_name);
                stage->setGoal(params.hand_open_pose);
                place->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
                stage->allowCollisions(params.object_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
                place->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
                stage->detachObject(params.object_name, params.hand_frame);
                place->insert(std::move(stage));
            }

            t.add(std::move(place));
        }

        /******************************************************
         * Move to Initialize                                 *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("move initialize", sampling_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setGoal(params.arm_initialize_pose);
            t.add(std::move(stage));
        }

        /******************************************************
         * Move to Home                                       *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setGoal(params.arm_home_pose);
            t.add(std::move(stage));
        }

        /******************************************************
         * FIXED: Closed gripper with Unique Name & correct property key
         * ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_close_pose);
            t.add(std::move(stage));
        }

        // Prepare Task structure for planning
        try {
            t.init();
        } catch (InitStageException& e) {
            RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
            return false;
        }

        return true;
    }

    bool PickPlaceTask::plan(const std::size_t max_solutions) {
        RCLCPP_INFO(LOGGER, "Start searching for task solutions");
        return static_cast<bool>(task_->plan(max_solutions));
    }

    bool PickPlaceTask::execute() {
        RCLCPP_INFO(LOGGER, "Executing solution trajectory");
        moveit_msgs::msg::MoveItErrorCodes execute_result;
        execute_result = task_->execute(*task_->solutions().front());

        if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
            return false;
        }
        return true;
    }
}