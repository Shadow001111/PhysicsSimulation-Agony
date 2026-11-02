#pragma once
#include "Editor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace PS_AGONY_EDITOR
{
	namespace Internal
	{
		constexpr ImU32 kSelectColor = IM_COL32(255, 220, 40, 255);
		constexpr ImU32 kGhostColor = IM_COL32(60, 255, 90, 255);
		constexpr ImU32 kHintColor = IM_COL32(255, 255, 255, 200);

		// TODO: express the pixel thresholds below as fractions of the screen resolution.
		constexpr Real kPickTolerancePx = 3;
		constexpr Real kMinDragPx = 4; // Dead zone for box select and launch.

		// Wrapped, dimmed help row.
		inline void hintRow(const char* text)
		{
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			ImGui::TextWrapped("%s", text);
			ImGui::PopStyleColor();
		}

		// Body-local (center of mass) <-> world helpers.
		template<class BV>
		inline Vec2 localToWorld(const BV& bv, ObjectDenseIndex bi, Vec2 l)
		{
			const Real c = bv.rotationCos[bi];
			const Real s = bv.rotationSin[bi];
			return Vec2(bv.worldCenterX[bi] + l.x * c - l.y * s, bv.worldCenterY[bi] + l.x * s + l.y * c);
		}

		template<class BV>
		inline Vec2 worldDirToLocal(const BV& bv, ObjectDenseIndex bi, Vec2 d)
		{
			const Real c = bv.rotationCos[bi];
			const Real s = bv.rotationSin[bi];
			return Vec2(d.x * c + d.y * s, -d.x * s + d.y * c);
		}

		inline bool dragVec2(const char* label, Vec2 v, Vec2& out, float speed = 0.01f)
		{
			float f[2] = { static_cast<float>(v.x), static_cast<float>(v.y) };
			if (!ImGui::DragFloat2(label, f, speed)) return false;
			out = Vec2(f[0], f[1]);
			return true;
		}

		inline bool dragReal(const char* label, Real v, Real& out, float speed, float lo, float hi)
		{
			float f = static_cast<float>(v);
			if (!ImGui::DragFloat(label, &f, speed, lo, hi)) return false;
			out = static_cast<Real>(f);
			return true;
		}
	}

	using namespace Internal;
}
