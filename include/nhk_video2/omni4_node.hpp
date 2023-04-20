#pragma once

#include <array>
#include <optional>
#include <string_view>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/Mechanism/omni_n.hpp>
#include <CRSLibtmp/Motor.hpp>
#include <CRSLibtmp/Ros2/can.hpp>
#include <CRSLibtmp/Ros2/reporter.hpp>

namespace NhkVideo2
{
	using namespace CRSLib::IntegerTypes;
	using CRSLib::Mechanism::OmniN;
	using CRSLib::Ros2::RosReporter;
	using CRSLib::Motor::ShirasuState;

	using CanPillarbox = CRSLib::Ros2::CanPillarbox;
	using Shirasu = CRSLib::Motor::Shirasu<CanPillarbox, RosReporter>;
	using OmniWheel = CRSLib::Mechanism::OmniWheel<Shirasu>;

	/// @attention constructor内で*thisを使っているので継承は強く非推奨。
	class Omni4Node final : public rclcpp::Node
	{
		std::optional<OmniN<OmniWheel, OmniWheel, OmniWheel, OmniWheel>> omni4{};
		rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr body_speed_sub{};
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enable_sub{};

		struct OmniWheelArg final
		{
			CRSLib::Math::Pose2D pose;
			double reduction_ratio;
			double wheel_radius;
			u32 base_id;

			#if 0 //adadhochoc!
			void read(Omni4Node& self, const std::string_view key)
			{
				int base_id_;
				if
				(
					!self.get_parameter_or<double>(std::string{key} + ".pose.x", pose.point[0], 1.0) ||
					!self.get_parameter_or<double>(std::string{key} + ".pose.y", pose.point[1], 1.0) ||
					!self.get_parameter_or<double>(std::string{key} + ".pose.theta", pose.theta, 0.0) ||
					!self.get_parameter_or<double>(std::string{key} + ".reduction_ratio", reduction_ratio, 1.0) ||
					!self.get_parameter_or<double>(std::string{key} + ".wheel_radius", wheel_radius, 100.0) ||
					!self.get_parameter_or<int>(std::string{key} + ".base_id", base_id_, 0x7FF)
				)
				{
					RCLCPP_ERROR(self.get_logger(), "Fail to read some argument.");
				}

				base_id = base_id_;
			}
			#endif
		};

		public:
		Omni4Node(const rclcpp::NodeOptions& options):
			rclcpp::Node("omni4", options)
		{
			body_speed_sub = this->create_subscription<geometry_msgs::msg::Pose2D>("body_speed", 1, std::bind(&Omni4Node::callback, this, std::placeholders::_1));
			enable_sub = this->create_subscription<std_msgs::msg::Empty>("enable_omni4", 1, std::bind(&Omni4Node::enable_callback, this, std::placeholders::_1));

			RCLCPP_INFO(this->get_logger(), "Omni4 initialize finish.");
		}
		
		void enable_callback(std_msgs::msg::Empty)
		{
			std::array<OmniWheelArg, 4> omni_wheel_args{};

			#if 0 // adadhochoc!
			std::array<std::string_view, 4> keys{"FrontLeft", "BackLeft", "BackRight", "FrontRight"};
			// パラメータから読み出し
			[&omni_wheel_args, keys]<size_t ... indices>(Omni4Node& self, std::index_sequence<indices ...>)
			{
				(omni_wheel_args[indices].read(self, keys[indices]), ...);
			}(*this, std::make_index_sequence<4>{});
			#else
			/// @todo 各種設定
			omni_wheel_args[0] = OmniWheelArg{.pose={.point={0.4053625, 0.4053625}, .theta=std::numbers::pi / 4}, .reduction_ratio=61.0, .wheel_radius=0.127, .base_id=0x200};
			omni_wheel_args[1] = OmniWheelArg{.pose={.point={-0.4053625, 0.4053625}, .theta=std::numbers::pi * 3 / 4}, .reduction_ratio=61.0, .wheel_radius=0.127, .base_id=0x150};
			omni_wheel_args[2] = OmniWheelArg{.pose={.point={-0.4053625, -0.4053625}, .theta=std::numbers::pi * 5 / 4}, .reduction_ratio=61.0, .wheel_radius=0.127, .base_id=0x210};
			omni_wheel_args[3] = OmniWheelArg{.pose={.point={0.4053625, -0.4053625}, .theta=std::numbers::pi * 7 / 4}, .reduction_ratio=61.0, .wheel_radius=0.127, .base_id=0x110};
			#endif

			// OmniWheelの作成
			constexpr auto make_omni_wheel = [](OmniWheelArg&& arg, Omni4Node& self) -> OmniWheel
			{
				constexpr auto make_logger = [](Omni4Node& self)
				{
					return RosReporter{self.get_logger(), rclcpp::Logger::Level::Error};
				};

				CanPillarbox command{self, arg.base_id, 100};
				CanPillarbox target{self, arg.base_id + 1, 100};
				Shirasu shirasu{std::move(command), std::move(target), make_logger(self)};
				shirasu.change_state(ShirasuState::velocity);
				return OmniWheel{std::move(shirasu), std::move(arg.pose), arg.reduction_ratio, arg.wheel_radius};
			};
			
			// 作成を実行
			[&omni_wheel_args, make_omni_wheel]<size_t ... indices>(Omni4Node& self, std::index_sequence<indices ...>)
			{
				self.omni4.emplace(make_omni_wheel(std::move(omni_wheel_args[indices]), self) ...);
			}(*this, std::make_index_sequence<4>{});
		}

		void callback(const geometry_msgs::msg::Pose2D& msg_pose)
		{
			if(omni4) omni4->update(CRSLib::Math::Pose2D{{msg_pose.x, msg_pose.y}, msg_pose.theta});
		}
	};
}