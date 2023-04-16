#pragma once

#include <cstring>
#include <type_traits>
#include <concepts>
#include <optional>
#include <memory>
#include <atomic>
#include <utility>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/reporter.hpp>
#include <CRSLibtmp/Can/MainPC/pillarbox.hpp>
#include <CRSLibtmp/Can/MainPC/letterbox.hpp>

namespace NhkVideo2
{
	using namespace CRSLib::IntegerTypes;

	enum class InjectorState : u8
	{
		Busy,
		Free
	};

	namespace Implement
	{
		struct InjectMotorMarker
		{};

		struct Callback final
		{
			std::atomic<InjectorState>& state;

			Callback(std::atomic<InjectorState>& state):
				state{state}
			{}

			void callback(const CRSLib::Can::DataField&) noexcept
			{
				state = InjectorState::Free;
			}
		};
	}

	template<class T>
	concept inject_motor = std::derived_from<T, Implement::InjectMotorMarker>;

	template<CRSLib::reporter Reporter, CRSLib::Can::MainPC::pillarbox CanPillarbox, CRSLib::Can::MainPC::letterbox CanLetterbox>
	struct InjectMotor final : Implement::InjectMotorMarker
	{
		Reporter reporter;
		CanPillarbox pillarbox;
		std::optional<CanLetterbox> letterbox{};
		std::atomic<InjectorState> state{InjectorState::Free};
		std::shared_ptr<Implement::Callback> callback{std::make_shared<Implement::Callback>(state)};

		InjectMotor(Reporter&& reporter, CanPillarbox&& pillarbox, CRSLib::Can::MainPC::letterbox_maker auto&& letterbox_maker)
		:
			reporter{std::move(reporter)},
			pillarbox{std::move(pillarbox)}
		{
			letterbox.emplace(letterbox_maker(callback));
		}

		// callbackがあり、これが自身への参照を持っているので無理。
		InjectMotor(InjectMotor&& other) = delete;

		void inject(const float rpm)
		{
			if(state == InjectorState::Free)
			{
				state = InjectorState::Busy;
				CRSLib::Can::DataField data{.dlc=sizeof(float)};
				std::memcpy(data.buffer, &rpm, sizeof(rpm));
				pillarbox.post(data);
			}
			else
			{
				reporter("Cannot start inject while injecting.");
			}
		}

		auto& get_state()
		{
			return state;
		}
	};

	template<CRSLib::reporter Reporter, CRSLib::Can::MainPC::pillarbox CanPillarbox>
	auto make_inject_motor(Reporter&& reporter, CanPillarbox&& pillarbox, CRSLib::Can::MainPC::letterbox_maker auto&& letterbox_maker)
	requires std::is_rvalue_reference_v<decltype(reporter)> && std::is_rvalue_reference_v<decltype(pillarbox)> && std::is_rvalue_reference_v<decltype(letterbox_maker)>
	{
		return InjectMotor<Reporter, CanPillarbox, decltype(letterbox_maker(std::declval<std::shared_ptr<Implement::Callback>>()))>{std::move(reporter), std::move(pillarbox), std::move(letterbox_maker)};
	}
}