#pragma once

#include <memory>

#include <CRSLibtmp/Ros2/logicool.hpp>
#include <CRSLibtmp/Ros2/can.hpp>
#include <CRSLibtmp/Ros2/reporter.hpp>
#include <CRSLibtmp/Motor/shirasu.hpp>
#include <CRSLibtmp/Motor/servo.hpp>
#include <CRSLibtmp/Mechanism/solenoid_valve.hpp>

#include <nhk_video2/inject_motor.hpp>
#include <nhk_video2/turnout_motor.hpp>
#include <nhk_video2/rokuro_kubi.hpp>
#include <nhk_video2/arm.hpp>
#include <nhk_video2/loader.hpp>
#include <nhk_video2/tusk.hpp>

namespace NhkVideo2
{
	struct Body final
	{
		using Reporter = CRSLib::Ros2::RosReporter;
		using CanPillarbox = CRSLib::Ros2::CanPillarbox;
		using CanLetterboxMaker = CRSLib::Ros2::CanLetterboxMaker;
		
		using Shirasu = CRSLib::Motor::Shirasu<CanPillarbox, Reporter>;
		using SolenoidValve = CRSLib::Mechanism::SolenoidValve<CanPillarbox>;

		/// @todo 方向の確認
		using RokuroKubi = NhkVideo2::RokuroKubi<SolenoidValve>;

		using ElbowGear = CRSLib::Mechanism::Gear<Shirasu>;
		/// @todo 方向の確認
		using Arm = NhkVideo2::Arm<Shirasu, ElbowGear, SolenoidValve>;

		using TurnoutMotor = NhkVideo2::TurnoutMotor<CanPillarbox>;
		/// @todo 方向の確認
		using Loader = NhkVideo2::Loader<TurnoutMotor, SolenoidValve>;

		using InjectMotor = decltype(NhkVideo2::make_inject_motor(std::declval<Reporter>(), std::declval<CanPillarbox>(), std::declval<CanLetterboxMaker>()));
		using YawGear = CRSLib::Mechanism::Gear<Shirasu>;
		using Tusk = NhkVideo2::Tusk<InjectMotor, YawGear>;
		
		struct Trunk final
		{
			std::unique_ptr<InjectMotor> inject_motor_up;
		};

		RokuroKubi rokuro_kubi;
		Arm arm;
		Loader loader;
		Tusk tusk_l;
		Tusk tusk_r;
		Trunk center_trunk;
	};
}