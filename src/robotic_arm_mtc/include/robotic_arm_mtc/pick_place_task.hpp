// ROS
#include <rclcpp/node.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <robotic_arm_mtc/pick_place_parameters.hpp>


#pragma once

namespace robotic_arm_mtc
 {
	using namespace moveit::task_constructor;

	// prepare a demo environment from ROS parameters under node
	void setupDemoScene(const pick_place_task_parameters::Params& params);

	class PickPlaceTask
	{
	public:
		PickPlaceTask(const std::string& task_name);
		~PickPlaceTask() = default;

		bool init(const rclcpp::Node::SharedPtr& node, const pick_place_task_parameters::Params& params);

		bool plan(const std::size_t max_solutions);

		bool execute();

	private:
		std::string task_name_;
		moveit::task_constructor::TaskPtr task_;
	};
} 