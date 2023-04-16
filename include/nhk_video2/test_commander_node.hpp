#pragma once

#include <cmath>
#include <optional>
#include <utility>
#include <ratio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <CRSLibtmp/std_type.hpp>
#include "body.hpp"

namespace NhkVideo2
{
	
	class TestCommanderNode : public rclcpp::Node
	{
		using Logicool = CRSLib::Ros2::Logicool;
		using KeyMap = CRSLib::Ros2::LogicoolXInputKeyMap;

		struct Constant
		{
			double body_rotation_speed = 60.0;
			double elbow_rotation_speed = 60.0;
			double lift_speed = 60.0;
			double tusk_yaw_gear_speed = 60.0;
			double injection_speed = 60.0;
		} constant{};
		
		std::optional<Logicool> logicool{};
		std::optional<Body> body{};

		rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr body_speed_pub{};
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inject_speed_pub{};
		rclcpp::TimerBase::SharedPtr timer{};

		public:
		TestCommanderNode(const rclcpp::NodeOptions& options):
			rclcpp::Node("test_commander", options)
		{
			{
				rclcpp::SubscriptionOptions options{};
				options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
				logicool.emplace(*this, "joy", options);
			}

			const auto make_reporter = [this]()
			{
				return Body::Reporter{this->get_logger(), rclcpp::Logger::Level::Error};
			};

			const auto make_shirasu = [this, make_reporter](const u32 id, const size_t command_queue_size, const size_t target_queue_size)
			{
				return Body::Shirasu{Body::CanPillarbox{*this, id, command_queue_size}, Body::CanPillarbox{*this, id + 1, target_queue_size}, make_reporter()};
			};

			const auto make_inject_motor_up = [this, make_reporter](Body::CanPillarbox&& pillar, Body::CanLetterboxMaker&& maker)
			{
				return std::make_unique<Body::InjectMotor>(make_reporter(), std::move(pillar), std::move(maker));
			};

			/// @todo constantの実行時の読み出し、bodyの実行時の情報による初期化

			{
				/// @todo idの設定
				Body::CanPillarbox kubi_pillar{*this, 0x7FF, 10};
				
				/// @todo 向きと初期伸縮
				Body::RokuroKubi kubi{Body::SolenoidValve{std::move(kubi_pillar)}};


				/// @todo idの設定
				auto arm_lift = make_shirasu(0x7FF, 10, 1);
				/// @todo idの設定
				auto elbow_motor = make_shirasu(0x7FF, 10, 1);
				Body::ElbowGear elbow{std::move(elbow_motor), 100, 0};
				/// @todo idの設定
				Body::CanPillarbox hand_pillar{*this, 0x7FF, 10};
				Body::SolenoidValve hand{std::move(hand_pillar)};

				Body::Arm arm{std::move(arm_lift), std::move(elbow), std::move(hand)};


				/// @todo idの設定
				Body::CanPillarbox loader_turnout_pillar{*this, 0x7FF, 10};
				Body::TurnoutMotor loader_turnout{std::move(loader_turnout_pillar)};
				/// @todo idの設定
				Body::CanPillarbox loader_cocking_pillar{*this, 0x7FF, 10};
				Body::SolenoidValve loader_cocking{std::move(loader_cocking_pillar)};
				
				Body::Loader loader{std::move(loader_turnout), std::move(loader_cocking)};


				rclcpp::SubscriptionOptions options{};
				options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

				/// @todo idの設定
				Body::CanPillarbox tusk_l_pillar{*this, 0x7FF, 10};
				/// @todo idの設定
				Body::CanLetterboxMaker tusk_l_letter_maker{*this, 0x7FF, 10, options};
				/// @todo idの設定
				auto tusk_l_gear_motor = make_shirasu(0x7FF, 10, 1);
				Body::YawGear tusk_l_gear{std::move(tusk_l_gear_motor), 100, 0};
				Body::Tusk tusk_l{make_inject_motor_up(std::move(tusk_l_pillar), std::move(tusk_l_letter_maker)), std::move(tusk_l_gear)};

				/// @todo idの設定
				Body::CanPillarbox tusk_r_pillar{*this, 0x7FF, 10};
				/// @todo idの設定
				Body::CanLetterboxMaker tusk_r_letter_maker{*this, 0x7FF, 10, options};
				/// @todo idの設定
				auto tusk_r_gear_motor = make_shirasu(0x7FF, 10, 1);
				Body::YawGear tusk_r_gear{std::move(tusk_r_gear_motor), 1800, 0};
				Body::Tusk tusk_r{make_inject_motor_up(std::move(tusk_r_pillar), std::move(tusk_r_letter_maker)), std::move(tusk_r_gear)};

				/// @todo idの設定
				Body::CanPillarbox trunk_pillar{*this, 0x7FF, 10};
				/// @todo idの設定
				Body::CanLetterboxMaker trunk_letter_maker{*this, 0x7FF, 10, options};
				Body::Trunk trunk{make_inject_motor_up(std::move(trunk_pillar), std::move(trunk_letter_maker))};


				body.emplace(std::move(kubi), std::move(arm), std::move(loader), std::move(tusk_l), std::move(tusk_r), std::move(trunk));
			}

			body_speed_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("body_speed", 1);
			inject_speed_pub = this->create_publisher<std_msgs::msg::Float32>("inject_speed", 1);

			using namespace std::chrono_literals;
			timer = this->create_wall_timer(1ms, std::bind(&TestCommanderNode::timer_callback, this));
		}

		private:
		void timer_callback()
		{
			// ここでlogicoolの値を読み取って、omni4に渡す
			{
				geometry_msgs::msg::Pose2D msg{};
				msg.x = -logicool->get_axis(KeyMap::Axes::l_stick_UD);
				msg.y = logicool->get_axis(KeyMap::Axes::l_stick_LR);
				msg.theta = logicool->is_being_pushed(KeyMap::Buttons::rb) ? constant.body_rotation_speed : logicool->is_being_pushed(KeyMap::Buttons::lb) ? -constant.body_rotation_speed : 0.0;

				if(msg.x != 0.0 || msg.y != 0.0 || msg.theta != 0.0)
				{
					body_speed_pub->publish(msg);
				}
			}

			// ろくろ首 射出と回収の切り替え
			{
				/// @todo 回収から装填機構へ受け渡しているときや、装填・射出をしているときに動かないようにする
				/// @todo 電磁弁のOpen/Closeとろくろ首の上下をそろえる
				if(logicool->is_pushed_down(KeyMap::Buttons::start))
				{
					body->rokuro_kubi.solenoid_valve.reverse();
				}
			}

			{
				// 回収する(ろくろ首が下がっている)
				if(body->rokuro_kubi.solenoid_valve.is_extend)
				{
					if(logicool->is_pushed_down(KeyMap::Buttons::a))
					{
						body->arm.hand.reverse();
					}

					const auto elbow_rotation_speed = logicool->get_axis(KeyMap::Axes::cross_LR) * constant.elbow_rotation_speed;
					const auto lift_speed = logicool->get_axis(KeyMap::Axes::cross_UD) * constant.lift_speed;

					if(elbow_rotation_speed != 0.0)
					{
						body->arm.elbow.update_speed(elbow_rotation_speed);
					}
					else if(lift_speed != 0.0)
					{
						body->arm.lift.update_speed(lift_speed);
					}
				}
				// 装填・射出する(ろくろ首が上がっている)
				else
				{
					enum class ChoseInjector : u8
					{
						none,
						left,
						right,
						center
					} chose_injector{ChoseInjector::none};

					if(const auto lr = logicool->get_axis(KeyMap::Axes::cross_LR))
					{
						/// @todo 向き確認
						chose_injector = std::signbit(lr) ? ChoseInjector::left : ChoseInjector::right;
					}
					else if(const auto ud = logicool->get_axis(KeyMap::Axes::cross_UD))
					{
						chose_injector = std::signbit(ud) ? ChoseInjector::center : ChoseInjector::none;
					}

					// 装填
					if(logicool->is_pushed_down(KeyMap::Buttons::y))
					{
						body->loader.cocking.extend();
						rclcpp::sleep_for(std::chrono::milliseconds(100));
						body->loader.cocking.contract();
					}

					// 射出
					const auto inject = [this](auto& injector)
					{
						const auto inject_speed = logicool->get_axis(KeyMap::Axes::r_stick_UD) * constant.injection_speed;

						std_msgs::msg::Float32 msg{};
						msg.data = inject_speed;
						inject_speed_pub->publish(msg);
						
						if(injector.inject_motor_up->get_state() == InjectorState::Free && logicool->is_being_pushed(KeyMap::Buttons::a))
						{
							injector.inject_motor_up->inject(inject_speed);
						}
					};

					const auto control_tusk = [this, inject](auto& tusk)
					{
						// ヨー角
						if(logicool->is_being_pushed(KeyMap::Buttons::lb))
						{
							tusk.yaw_gear.update_speed(-constant.tusk_yaw_gear_speed);
						}
						else if(logicool->is_being_pushed(KeyMap::Buttons::rb))
						{
							tusk.yaw_gear.update_speed(constant.tusk_yaw_gear_speed);
						}
						else
						{
							tusk.yaw_gear.update_speed(0.0);
						}

						inject(tusk);
					};

					switch(chose_injector)
					{
						case ChoseInjector::left:
						control_tusk(body->tusk_l);
						break;

						case ChoseInjector::right:
						control_tusk(body->tusk_r);
						break;

						case ChoseInjector::center:
						inject(body->center_trunk);
						break;

						default:;
					}
				}
			}
		}
	};
}