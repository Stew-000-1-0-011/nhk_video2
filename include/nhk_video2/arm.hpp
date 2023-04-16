#pragma once

#include <CRSLibtmp/Motor/speed_controlled.hpp>
#include <CRSLibtmp/Mechanism/gear.hpp>
#include <CRSLibtmp/Mechanism/solenoid_valve.hpp>

namespace NhkVideo2
{
	template<CRSLib::Motor::speed_controlled_motor Lift, CRSLib::Mechanism::gear Elbow, CRSLib::Mechanism::solenoid_valve Hand>
	struct Arm final
	{
		Lift lift;
		Elbow elbow;
		Hand hand;
	};
}