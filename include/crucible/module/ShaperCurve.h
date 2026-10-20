// SPDX-License-Identifier: MIT
#pragma once

// Capability interface of the waveshaper module (fx.shaper): the control thread hands it the transfer curve (see
// fx/ShaperCurve.h); hosts find it with ModuleRack::findCapability<>() and the engine publishes the curve as the
// "curve" view of the module.

#include <crucible/fx/ShaperCurve.h>

#include <string>
#include <vector>

namespace crucible
{

class ShaperCurve
{
public:
    // Points of the "curve" view: the curve sampled at x = -1 .. +1.
    static constexpr int kViewPoints = 257;

    virtual ~ShaperCurve() = default;
    // Control thread. Validates and publishes the curve; the audio thread takes it at the next block boundary and
    // cross-fades to it. False with the reason in error leaves the curve as it was.
    virtual bool setCurve(const ShaperPoint* points, int count, bool symmetric, std::string& error) = 0;
    // Back to the default identity curve (y = x).
    virtual void resetCurve() = 0;
    // The curve as given to setCurve(), for saving in a patch; false while it is the default identity.
    virtual bool curve(std::vector<ShaperPoint>& points, bool& symmetric) const = 0;
    // The "curve" view: kViewPoints values of the curve (without drive) at x = -1 .. +1, in steps of 2 / (kViewPoints - 1).
    // Any thread except the audio thread (a pulled view, computed by the reader).
    virtual void fillCurveView(float* out) = 0;
};

} // namespace crucible
