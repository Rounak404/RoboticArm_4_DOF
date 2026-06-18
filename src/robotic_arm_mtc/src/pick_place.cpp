

// ROS
#include <rclcpp/rclcpp.hpp>

// MTC pick/place demo implementation
#include <robotic_arm_mtc/pick_place_task.hpp>
#include <robotic_arm_mtc/pick_place_parameters.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robotic_arm_mtc");

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("moveit_task_constructor_demo", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	const auto param_listener = std::make_shared<pick_place_task_parameters::ParamListener>(node);
	const auto params = param_listener->get_params();
	robotic_arm_mtc::setupDemoScene(params);

	// Construct and run pick/place task
	robotic_arm_mtc::PickPlaceTask pick_place_task("pick_place_task");
	if (!pick_place_task.init(node, params)) {
		RCLCPP_INFO(LOGGER, "Initialization failed");
		return 1;
	}

	if (pick_place_task.plan(params.max_solutions)) {
		RCLCPP_INFO(LOGGER, "Planning succeded");
		if (params.execute) {
			pick_place_task.execute();
			RCLCPP_INFO(LOGGER, "Execution complete");
		} else {
			RCLCPP_INFO(LOGGER, "Execution disabled");
		}
	} else {
		RCLCPP_INFO(LOGGER, "Planning failed");
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}