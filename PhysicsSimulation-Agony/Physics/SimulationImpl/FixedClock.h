#pragma once
#include "Physics/Types.h"

#include <cmath>
#include <cstdint>

namespace PS_AGONY
{
    class FixedClock
    {
    public:
        struct Settings
        {
            Real updateInterval = Real(1) / Real(300);
            Real maxDeltaTimePerUpdateCall = Real(1) / Real(20);
            Real timeScale = Real(1);
        };

        FixedClock() = default;
        explicit FixedClock(const Settings& settings) noexcept :
            settings(settings)
        {
        }

        // Advances time, computes required sub-step count, and updates render alpha.
        uint32_t advance(Real deltaTime) noexcept
        {
            if (deltaTime <= Real(0))
            {
                currentStepCount = 0;
                return 0;
            }

            // Cap maximum consumed real time per call to prevent spiral-of-death.
            const Real cappedDeltaTime = std::fmin(deltaTime, settings.maxDeltaTimePerUpdateCall);
            accumulator += cappedDeltaTime;

            const uint32_t stepCount = static_cast<uint32_t>(std::floor(accumulator / settings.updateInterval));
            accumulator -= static_cast<Real>(stepCount) * settings.updateInterval;

            if (stepCount > 0)
            {
                lastStepCount = stepCount;
                totalSimulationTime += static_cast<Real>(stepCount) * getFixedDeltaTime();
            }

            currentStepCount = stepCount;

            // Calculate interpolation alpha across fixed steps.
            const Real rawAlpha = accumulator / settings.updateInterval;
            renderAlpha = (static_cast<Real>(lastStepCount - 1) + rawAlpha) / static_cast<Real>(lastStepCount);

            return stepCount;
        }

        void reset() noexcept
        {
            accumulator = Real(0);
            totalSimulationTime = Real(0);
            renderAlpha = Real(0);
            lastStepCount = 1;
            currentStepCount = 0;
        }

        // Getters
        [[nodiscard]] Real getFixedDeltaTime() const noexcept
        {
            return settings.updateInterval * settings.timeScale;
        }

        [[nodiscard]] uint32_t getCurrentStepCount() const noexcept { return currentStepCount; }
        [[nodiscard]] Real getRenderAlpha() const noexcept { return renderAlpha; }
        [[nodiscard]] Real getTotalSimulationTime() const noexcept { return totalSimulationTime; }
        [[nodiscard]] Real getAccumulator() const noexcept { return accumulator; }

        // Settings access
        [[nodiscard]] Settings& getSettings() noexcept { return settings; }
        [[nodiscard]] const Settings& getSettings() const noexcept { return settings; }
        void setSettings(const Settings& settings) noexcept { this->settings = settings; }

    private:
        Settings settings;

        Real accumulator = Real(0);
        Real totalSimulationTime = Real(0);
        Real renderAlpha = Real(0);

        uint32_t lastStepCount = 1;
        uint32_t currentStepCount = 0;
    };
}