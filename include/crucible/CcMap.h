// SPDX-License-Identifier: MIT
#pragma once

// MIDI CC mapping with learn: which controller (optionally on one channel) drives which parameter, how.
//
// A binding connects CC number + channel (-1 = any) to a parameter with a range (lo..hi of the normalised
// parameter, lo > hi inverts), a curve and a mode (absolute 0..127, or one of the three relative encodings that
// endless encoders send). Bindings are edited on a control thread and published as an immutable table that the
// MIDI / audio thread reads without locks.
//
// Learn: armLearn(param) waits for the next controller message (CCs 1, 64, 120 and 123 are reserved and ignored).
// The thread that receives MIDI calls offer(); it consumes the message, applies it to the parameter and leaves a
// note for the control thread, which turns it into a binding with pollLearned(). noteCc() records the last message
// for UIs ("move a control...").

#include <crucible/ParameterMap.h>
#include <crucible/RtShared.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kMaxCcBindings = 256;

enum class CcMode : int
{
    Absolute,
    RelativeTwos,      // 1..63 up, 127..65 down
    RelativeSignedBit, // 1..63 up, 65..127 down
    RelativeBinary     // 64 is zero, above up, below down
};

struct CcBinding
{
    int cc = 0;
    int channel = -1; // 0..15, -1 = any
    std::string param;
    float lo = 0.0f; // normalised range
    float hi = 1.0f;
    float curve = 0.0f; // -1..1, 0 = linear
    CcMode mode = CcMode::Absolute;
    float step = 1.0f / 64.0f; // relative modes: normalised change per tick
};

struct CcTable
{
    std::vector<CcBinding> bindings;
    std::vector<int> paramIndex; // per binding, -1 for unknown parameters
};

// New normalised value of the parameter for a controller message. `current` is the parameter's present normalised
// value (used by relative modes).
float applyCcBinding(const CcBinding& binding, int data2, float current);
bool isReservedCc(int cc);

struct CcLearned
{
    int cc = 0;
    int channel = 0;
    int paramIndex = -1;
    float value = 0.0f;
};

class CcMapStore
{
public:
    // model: parameter model of the product (must outlive the store); null = process default (deprecated shim).
    explicit CcMapStore(const ParameterModel* model = nullptr);

    // ---- Control thread.
    bool add(const CcBinding& binding); // replaces the binding with the same cc, channel and parameter
    bool remove(int cc, int channel, const std::string& param);
    int removeParam(const std::string& param); // unlearn: returns how many bindings went
    int removeCc(int cc, int channel);
    void clear();
    std::vector<CcBinding> bindings() const;
    std::vector<CcBinding> bindingsFor(const std::string& param) const;
    void replaceAll(const std::vector<CcBinding>& bindings);
    void collectGarbage() { shared.collectGarbage(); }

    // "cc,channel,param,lo,hi,curve,mode,step" rows joined with ';'.
    std::string serialise() const;
    bool deserialise(const std::string& text);

    // Learn. `templateBinding` supplies range, curve, mode and step of the new binding; channelSpecific binds the
    // channel the message arrived on instead of any channel.
    bool armLearn(const std::string& param, bool channelSpecific = false, const CcBinding& templateBinding = {});
    void cancelLearn();
    bool learning() const { return pendingParam.load(std::memory_order_acquire) >= 0; }
    // Creates the binding for a learned message. Returns true when one was added.
    bool pollLearned(CcBinding* added = nullptr);

    // ---- MIDI / audio thread.
    // True when the message was consumed by learn (apply `out.value` to out.paramIndex).
    bool offer(int channel, int cc, int data2, CcLearned& out) noexcept;
    void noteCc(int channel, int cc, int data2) noexcept;
    // Last controller message seen: cc, channel and 0..127 value; false when none yet.
    bool lastCc(int& cc, int& channel, int& value) const noexcept;
    const CcTable* acquire() noexcept { return shared.acquire(); }
    void release() noexcept { shared.release(); }

private:
    void publishLocked();
    const ParameterModel* params;
    mutable std::mutex mutex;
    std::vector<CcBinding> list;
    RtShared<CcTable> shared;

    std::atomic<int> pendingParam{-1};
    std::atomic<int> pendingChannelSpecific{0};
    std::atomic<std::uint32_t> learned{0}; // bit 31 valid, 30..24 cc, 23..19 channel+1... see .cpp
    std::atomic<std::uint32_t> last{0};
    CcBinding learnTemplate;
    std::string learnParam;
};

} // namespace crucible
