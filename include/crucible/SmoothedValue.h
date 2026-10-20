// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace crucible
{

// Linear ramp towards a target over a fixed number of steps. Allocation free; one instance per smoothed value.
// Semantics match the common audio-framework linear smoother: a new target restarts the full ramp from the current
// value, a target that is (relatively) equal to the current target is ignored, and a ramp of zero steps jumps.
class LinearSmoothedValue
{
public:
    LinearSmoothedValue() = default;
    explicit LinearSmoothedValue(float initial) : current(initial), target(initial) {}

    // Ramp length in seconds at the given update rate (calls to getNextValue() per second). Snaps to the target.
    void reset(double updateRate, double rampSeconds) noexcept
    {
        reset(static_cast<int>(std::floor(rampSeconds * updateRate)));
    }

    void reset(int numSteps) noexcept
    {
        stepsToTarget = numSteps;
        setCurrentAndTargetValue(target);
    }

    void setCurrentAndTargetValue(float v) noexcept
    {
        target = current = v;
        countdown = 0;
    }

    void setTargetValue(float v) noexcept
    {
        if (approximatelyEqual(v, target))
            return;
        if (stepsToTarget <= 0)
        {
            setCurrentAndTargetValue(v);
            return;
        }
        target = v;
        countdown = stepsToTarget;
        step = (target - current) / static_cast<float>(countdown);
    }

    float getNextValue() noexcept
    {
        if (!isSmoothing())
            return target;
        --countdown;
        if (isSmoothing())
            current += step;
        else
            current = target;
        return current;
    }

    bool isSmoothing() const noexcept { return countdown > 0; }
    float getCurrentValue() const noexcept { return current; }
    float getTargetValue() const noexcept { return target; }

private:
    static bool approximatelyEqual(float a, float b) noexcept
    {
        if (!(std::isfinite(a) && std::isfinite(b)))
            return std::equal_to<float>{}(a, b); // non-finite: exact comparison
        const float diff = std::abs(a - b);
        return diff <= std::numeric_limits<float>::min() ||
               diff <= std::numeric_limits<float>::epsilon() * std::max(std::abs(a), std::abs(b));
    }

    float current = 0.0f;
    float target = 0.0f;
    float step = 0.0f;
    int countdown = 0;
    int stepsToTarget = 0;
};

} // namespace crucible
