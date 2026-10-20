// SPDX-License-Identifier: MIT
#pragma once

// Capability interface of the convolution module (fx.convolution): the control thread hands it an imported impulse
// response (mono, stereo or true-stereo, see fx/IrImport.h); hosts find it with ModuleRack::findCapability<>().

#include <crucible/fx/IrImport.h>

#include <string>

namespace crucible
{

class ConvolutionIr
{
public:
    virtual ~ConvolutionIr() = default;
    // Control thread. Copies the response (any sample rate; it is resampled to the engine rate when the kernel is built).
    // The kernel is built off the audio thread and crossfaded in. Returns false when the response is empty.
    virtual bool setImpulseResponse(const ImpulseResponse& ir, std::string name) = 0;
    virtual void clearImpulseResponse() = 0;
    virtual bool hasImpulseResponse() const = 0;
    virtual std::string impulseResponseName() const = 0;
    // The response as loaded (before edits and resampling), for saving in a patch; false when none.
    virtual bool impulseResponse(ImpulseResponse& out, std::string& name) const = 0;
};

} // namespace crucible
