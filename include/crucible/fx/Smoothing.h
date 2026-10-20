// SPDX-License-Identifier: MIT
#pragma once

// Parameter smoothing for effects (docs/plans/ENGINE-FX-PLAN.md, sections 3.1 and 4.7). The engine hands a module one
// value per control block; these helpers turn it into a per-sample path with no steps:
//
//   LinearRamp     gains: a straight line from the previous value to the new one across the block
//   LogRamp        frequencies: a geometric ramp, so an octave sweep moves at constant speed
//   OnePole        times and slow tracking: exponential approach with a time constant in seconds
//   Crossfader     discrete switches (model, algorithm, stage count): twin-state fade between old and new kernels
//
// Nothing here allocates; every call is audio-thread safe.

#include <algorithm>
#include <cmath>

namespace crucible::fx
{

// One float one-pole step that cannot stall: once coef * (target - v) is too small to move v (it rounds to nothing, well
// before any relative-epsilon snap would fire) the value lands on the target. A smoothed control therefore always ends
// exactly on its target, so a constant setting returns to its static path.
inline float onePoleStepSnap(float v, float target, float coef)
{
    const float next = v + coef * (target - v);
    return next == v ? target : next;
}

// Quintic "smootherstep": 0 at 0, 1 at 1, with zero first and second derivative at both ends.
inline float smootherstep(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

class LinearRamp
{
public:
    void reset(float value)
    {
        current = target = value;
        step = 0.0f;
        remaining = 0;
    }
    // Reach `newTarget` exactly after `samples` calls of next(). samples <= 0 jumps.
    void setTarget(float newTarget, int samples)
    {
        target = newTarget;
        if (samples <= 0)
        {
            current = newTarget;
            step = 0.0f;
            remaining = 0;
            return;
        }
        step = (newTarget - current) / static_cast<float>(samples);
        remaining = samples;
    }
    float next()
    {
        if (remaining > 0)
        {
            current += step;
            if (--remaining == 0)
                current = target;
        }
        return current;
    }
    void fill(float* out, int n)
    {
        for (int i = 0; i < n; ++i)
            out[i] = next();
    }
    float value() const { return current; }
    float targetValue() const { return target; }
    bool ramping() const { return remaining > 0; }

private:
    float current = 0.0f, target = 0.0f, step = 0.0f;
    int remaining = 0;
};

// Geometric ramp for strictly positive values (frequencies in Hz).
class LogRamp
{
public:
    void reset(float value)
    {
        current = target = std::max(value, 1.0e-9f);
        ratio = 1.0f;
        remaining = 0;
    }
    void setTarget(float newTarget, int samples)
    {
        target = std::max(newTarget, 1.0e-9f);
        if (samples <= 0)
        {
            current = target;
            ratio = 1.0f;
            remaining = 0;
            return;
        }
        ratio = std::pow(target / current, 1.0f / static_cast<float>(samples));
        remaining = samples;
    }
    float next()
    {
        if (remaining > 0)
        {
            current *= ratio;
            if (--remaining == 0)
                current = target;
        }
        return current;
    }
    float value() const { return current; }
    bool ramping() const { return remaining > 0; }

private:
    float current = 1.0f, target = 1.0f, ratio = 1.0f;
    int remaining = 0;
};

// One-pole approach to a target; timeSeconds is the 63 % time. A time of 0 follows the target at once.
class OnePole
{
public:
    void setTime(float timeSeconds, double sampleRate)
    {
        coefficient = timeSeconds <= 0.0f
                          ? 1.0f
                          : 1.0f - static_cast<float>(std::exp(-1.0 / (static_cast<double>(timeSeconds) * sampleRate)));
    }
    void reset(float value) { state = value; }
    float next(float target)
    {
        state += (target - state) * coefficient;
        return state;
    }
    float value() const { return state; }

private:
    float coefficient = 1.0f, state = 0.0f;
};

// Twin-state crossfade for discrete changes. The owner keeps two kernels; while active() it runs both and mixes
// old * (1 - w) + new * w with w = next(). When the fade ends the old kernel can be recycled. start() while a fade is
// running is refused (returns false) so that the caller can queue the latest request.
class Crossfader
{
public:
    void prepare(double sampleRate, double seconds = 0.020)
    {
        length = std::max(1, static_cast<int>(std::lround(seconds * sampleRate)));
        position = length;
    }
    bool start()
    {
        if (active())
            return false;
        position = 0;
        return true;
    }
    bool active() const { return position < length; }
    // Weight of the new kernel for the next sample, in [0, 1]. Returns 1 when idle.
    float next()
    {
        if (!active())
            return 1.0f;
        const float w = smootherstep(static_cast<float>(position + 1) / static_cast<float>(length));
        ++position;
        return w;
    }
    int lengthSamples() const { return length; }

private:
    int length = 1, position = 1;
};

} // namespace crucible::fx
