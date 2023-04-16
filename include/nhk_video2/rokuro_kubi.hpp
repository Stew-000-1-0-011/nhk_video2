#pragma once

#include <type_traits>
#include <CRSLibtmp/Can/MainPC/pillarbox.hpp>
#include <CRSLibtmp/Mechanism/solenoid_valve.hpp>

namespace NhkVideo2
{
	template<CRSLib::Mechanism::solenoid_valve SolenoidValve>
	struct RokuroKubi final
	{
		SolenoidValve solenoid_valve;
	};
}