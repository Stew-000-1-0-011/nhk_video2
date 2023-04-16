#pragma once

#include <CRSLibtmp/Mechanism/solenoid_valve.hpp>
#include "turnout_motor.hpp"

namespace NhkVideo2
{
	template<turnout_motor Turnout, CRSLib::Mechanism::solenoid_valve Cocking>
	struct Loader final
	{
		Turnout turnout;
		Cocking cocking;
	};
}