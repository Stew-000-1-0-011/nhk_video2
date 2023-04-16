#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nhk_video2/omni4_node.hpp>
#include <nhk_video2/test_commander_node.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options;

	auto omni4 = std::make_shared<NhkVideo2::Omni4Node>(options);
	auto test_commander = std::make_shared<NhkVideo2::TestCommanderNode>(options); 
	exec.add_node(omni4);
	exec.add_node(test_commander);
	
	std::cout << "tamaki started." << std::endl;

	exec.spin();
	
	std::cout << "tamaki ended." << std::endl;

	rclcpp::shutdown();

	return 0;
}
