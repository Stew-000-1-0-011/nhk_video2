#pragma once

#include <type_traits>
#include <concepts>
#include <utility>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/Can/MainPC/pillarbox.hpp>

namespace NhkVideo2
{
	using namespace CRSLib::IntegerTypes;

	namespace Implement
	{
		struct InjectMotorMarker
		{};
	}

	template<class T>
	concept inject_motor = std::derived_from<T, Implement::InjectMotorMarker>;

	template<CRSLib::Can::MainPC::pillarbox CanPillarbox>
	struct InjectMotor final : Implement::InjectMotorMarker
	{
		CanPillarbox pillarbox;

		InjectMotor(CanPillarbox&& pillarbox)
		:
			pillarbox{std::move(pillarbox)}
		{}

		void inject()
		{
			CRSLib::Can::DataField data{.buffer={byte{}}, .dlc=1};
			pillarbox.post(data);
		}
	};
}