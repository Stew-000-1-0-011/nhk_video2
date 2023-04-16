#include <rclcpp/rclcpp.hpp>
#include <nhk_video2/omni4_node.hpp>
#include <nhk_video2/test_commander_node.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options;

	exec.add_node(std::make_shared<NhkVideo2::Omni4Node>(options));
	exec.add_node(std::make_shared<NhkVideo2::TestCommanderNode>(options));

	exec.spin();

	rclcpp::shutdown();

	return 0;
}