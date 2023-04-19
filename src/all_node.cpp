#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <joy/joy.hpp>
#include <nhk_video2/omni4_node.hpp>
#include <nhk_video2/test_commander_node.hpp>

int main(int argc, char * argv[])
{
	std::cout << "tamaki started main." << std::endl;

	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options;

	std::cout << "tamaki started initilalizing nodes." << std::endl;

	auto omni4 = std::make_shared<NhkVideo2::Omni4Node>(options);
	auto test_commander = std::make_shared<NhkVideo2::TestCommanderNode>(options); 
	auto joy_ = std::make_shared<joy::Joy>(options);
	exec.add_node(omni4);
	exec.add_node(test_commander);
	exec.add_node(joy_);
	
	std::cout << "tamaki started spinning." << std::endl;

	exec.spin();
	
	std::cout << "tamaki ended." << std::endl;

	rclcpp::shutdown();
}
