#pragma once

#include <type_traits>
#include <concepts>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/reporter.hpp>
#include <CRSLibtmp/Can/utility.hpp>
#include <CRSLibtmp/Can/MainPC/pillarbox.hpp>

namespace NhkVideo2
{
	using namespace CRSLib::IntegerTypes;

	enum class LoadDestination : u8
	{
		LelfTusk,
		RightTusk,
		Trunk
	};

	namespace Implement
	{
		struct TurnoutMotorMarker
		{};
	}

	template<class T>
	concept turnout_motor = std::derived_from<T, Implement::TurnoutMotorMarker>;

	template<CRSLib::Can::MainPC::pillarbox CanPillarbox>
	struct TurnoutMotor final : Implement::TurnoutMotorMarker
	{
		CanPillarbox pillarbox;

		TurnoutMotor(CanPillarbox&& pillarbox)
		requires std::is_rvalue_reference_v<decltype(pillarbox)>
		:
			pillarbox{std::move(pillarbox)}
		{}

		void turn(const LoadDestination destination)
		{
			pillarbox.post(CRSLib::Can::DataField{.buffer={static_cast<byte>(destination)}, .dlc=1});
		}
	};
}