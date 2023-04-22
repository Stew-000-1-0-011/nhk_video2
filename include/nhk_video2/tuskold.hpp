#pragma once

#include <memory>

#include <CRSLibtmp/Mechanism/gear.hpp>
#include "inject_motor.hpp"

namespace NhkVideo2
{
	template<inject_motor InjectMotor, CRSLib::Mechanism::gear YawGear>
	struct Tusk final
	{
		std::unique_ptr<InjectMotor> inject_motor_up;
		YawGear yaw_gear;

		Tusk(std::unique_ptr<InjectMotor>&& inject_motor_up, YawGear&& yaw_gear)
		requires std::is_rvalue_reference_v<decltype(yaw_gear)>
		:
			inject_motor_up{std::move(inject_motor_up)},
			yaw_gear{std::move(yaw_gear)}
		{}
	};
}