// SPDX-License-Identifier: MIT
#pragma once

// Test hooks of fx.cabinet (found with ModuleRack::findCapability<CabinetProbe>("fx.cabinet")). They let a test hold the
// kernel builder at the moment a finished set would be published, and count the kernel sets that are alive, so races
// between a rate change and a build in flight and the retirement of sets can be measured. Hosts have no use for it.

namespace crucible
{

class CabinetProbe
{
public:
    virtual ~CabinetProbe() = default;

    // While held, the builder still builds but does not publish the finished set and does not free retired sets.
    virtual void holdBuilder(bool hold) = 0;
    // True while a finished set waits to be taken by the audio thread.
    virtual bool setPending() const = 0;
    // Sample rate of the kernel set that is playing, priming or fading out (0 when none); the last one found.
    virtual double playingRate() const = 0;
    // Retires `count` empty kernel sets from the calling thread (as the audio thread retires replaced sets).
    virtual void retireSpares(int count) = 0;
    // Kernel sets alive in the process (all cabinet instances).
    static int liveSets();
};

} // namespace crucible
