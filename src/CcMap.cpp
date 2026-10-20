// SPDX-License-Identifier: MIT
#include <crucible/CcMap.h>
#include <crucible/module/Module.h>

#include <crucible/ParameterMap.h>

#include "TextFormat.h"

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
// learned: bit 31 valid | bits 30..24 cc | bit 23 channel-specific | bits 22..19 channel | bits 18..12 value | 11..0 param
std::uint32_t packLearned(int cc, bool specific, int channel, int value, int param)
{
    return 0x80000000U | (static_cast<std::uint32_t>(cc & 0x7f) << 24) | (specific ? 0x00800000U : 0U) |
           (static_cast<std::uint32_t>(channel & 0xf) << 19) | (static_cast<std::uint32_t>(value & 0x7f) << 12) |
           static_cast<std::uint32_t>(param & 0xfff);
}
} // namespace

bool isReservedCc(int cc)
{
    return cc == 1 || cc == 64 || cc == 120 || cc == 123;
}

float applyCcBinding(const CcBinding& b, int data2, float current)
{
    const float lo = std::min(b.lo, b.hi), hi = std::max(b.lo, b.hi);
    const int v = data2 & 0x7f;
    float n;
    switch (b.mode)
    {
    case CcMode::Absolute:
    {
        float x = static_cast<float>(v) / 127.0f;
        if (b.curve != 0.0f)
            x = std::pow(x, std::exp2(b.curve * 2.0f));
        n = b.lo + (b.hi - b.lo) * x;
        return std::clamp(n, lo, hi);
    }
    case CcMode::RelativeTwos:
        n = static_cast<float>(v < 64 ? v : v - 128);
        break;
    case CcMode::RelativeSignedBit:
        n = static_cast<float>(v < 64 ? v : -(v - 64));
        break;
    case CcMode::RelativeBinary:
        n = static_cast<float>(v - 64);
        break;
    default:
        n = 0.0f;
        break;
    }
    return std::clamp(current + n * b.step * (b.hi >= b.lo ? 1.0f : -1.0f), lo, hi);
}

CcMapStore::CcMapStore(const ParameterModel* model)
    : params(model != nullptr ? model : &ModuleRegistry::instance().parameters())
{
    shared.publish(std::make_unique<CcTable>());
}

void CcMapStore::publishLocked()
{
    auto table = std::make_unique<CcTable>();
    table->bindings = list;
    for (const auto& b : list)
        table->paramIndex.push_back(params->indexOf(b.param));
    shared.publish(std::move(table));
}

bool CcMapStore::add(const CcBinding& binding)
{
    if (binding.cc < 0 || binding.cc > 127 || isReservedCc(binding.cc) || binding.channel < -1 || binding.channel > 15 ||
        params->indexOf(binding.param) < 0)
        return false;
    CcBinding b = binding;
    b.lo = std::clamp(b.lo, 0.0f, 1.0f);
    b.hi = std::clamp(b.hi, 0.0f, 1.0f);
    b.curve = std::clamp(b.curve, -1.0f, 1.0f);
    b.step = std::clamp(b.step, 1.0e-4f, 1.0f);
    if (static_cast<int>(b.mode) < 0 || static_cast<int>(b.mode) > 3)
        b.mode = CcMode::Absolute;
    std::lock_guard<std::mutex> lock(mutex);
    for (auto& e : list)
        if (e.cc == b.cc && e.channel == b.channel && e.param == b.param)
        {
            e = b;
            publishLocked();
            return true;
        }
    if (static_cast<int>(list.size()) >= kMaxCcBindings)
        return false;
    list.push_back(b);
    publishLocked();
    return true;
}

bool CcMapStore::remove(int cc, int channel, const std::string& param)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto before = list.size();
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](const CcBinding& b) { return b.cc == cc && b.channel == channel && b.param == param; }),
               list.end());
    if (list.size() == before)
        return false;
    publishLocked();
    return true;
}

int CcMapStore::removeParam(const std::string& param)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto before = list.size();
    list.erase(std::remove_if(list.begin(), list.end(), [&](const CcBinding& b) { return b.param == param; }), list.end());
    if (list.size() != before)
        publishLocked();
    return static_cast<int>(before - list.size());
}

int CcMapStore::removeCc(int cc, int channel)
{
    std::lock_guard<std::mutex> lock(mutex);
    const auto before = list.size();
    list.erase(
        std::remove_if(list.begin(), list.end(), [&](const CcBinding& b) { return b.cc == cc && b.channel == channel; }),
        list.end());
    if (list.size() != before)
        publishLocked();
    return static_cast<int>(before - list.size());
}

void CcMapStore::clear()
{
    std::lock_guard<std::mutex> lock(mutex);
    list.clear();
    publishLocked();
}

std::vector<CcBinding> CcMapStore::bindings() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return list;
}

std::vector<CcBinding> CcMapStore::bindingsFor(const std::string& param) const
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<CcBinding> out;
    for (const auto& b : list)
        if (b.param == param)
            out.push_back(b);
    return out;
}

void CcMapStore::replaceAll(const std::vector<CcBinding>& bindings)
{
    clear();
    for (const auto& b : bindings)
        add(b);
}

std::string CcMapStore::serialise() const
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::string> rows;
    for (const auto& b : list)
        rows.push_back(std::to_string(b.cc) + "," + std::to_string(b.channel) + "," + b.param + "," + text::fixed(b.lo, 6) +
                       "," + text::fixed(b.hi, 6) + "," + text::fixed(b.curve, 6) + "," +
                       std::to_string(static_cast<int>(b.mode)) + "," + text::fixed(b.step, 6));
    return text::join(rows, ';');
}

bool CcMapStore::deserialise(const std::string& textIn)
{
    std::vector<CcBinding> parsed;
    for (const auto& row : text::split(textIn, ';'))
    {
        const auto f = text::split(row, ',');
        if (f.size() < 3)
            continue;
        CcBinding b;
        b.cc = text::toInt(f[0]);
        b.channel = text::toInt(f[1]);
        b.param = f[2];
        if (f.size() > 3)
            b.lo = text::toFloat(f[3]);
        if (f.size() > 4)
            b.hi = text::toFloat(f[4]);
        if (f.size() > 5)
            b.curve = text::toFloat(f[5]);
        if (f.size() > 6)
            b.mode = static_cast<CcMode>(text::toInt(f[6]));
        if (f.size() > 7)
            b.step = text::toFloat(f[7]);
        parsed.push_back(b);
    }
    replaceAll(parsed);
    return true;
}

bool CcMapStore::armLearn(const std::string& param, bool channelSpecific, const CcBinding& templateBinding)
{
    const int idx = params->indexOf(param);
    if (idx < 0 || idx > 0xfff)
        return false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        learnTemplate = templateBinding;
        learnParam = param;
    }
    learned.store(0, std::memory_order_release);
    pendingChannelSpecific.store(channelSpecific ? 1 : 0, std::memory_order_relaxed);
    pendingParam.store(idx, std::memory_order_release);
    return true;
}

void CcMapStore::cancelLearn()
{
    pendingParam.store(-1, std::memory_order_release);
}

bool CcMapStore::pollLearned(CcBinding* added)
{
    const std::uint32_t v = learned.exchange(0, std::memory_order_acq_rel);
    if ((v & 0x80000000U) == 0)
        return false;
    CcBinding b;
    {
        std::lock_guard<std::mutex> lock(mutex);
        b = learnTemplate;
    }
    b.cc = static_cast<int>((v >> 24) & 0x7fU);
    b.channel = (v & 0x00800000U) != 0 ? static_cast<int>((v >> 19) & 0xfU) : -1;
    const int idx = static_cast<int>(v & 0xfffU);
    const auto& defs = params->params();
    if (static_cast<size_t>(idx) >= defs.size())
        return false;
    b.param = defs[static_cast<size_t>(idx)].id;
    if (!add(b))
        return false;
    if (added != nullptr)
        *added = b;
    return true;
}

bool CcMapStore::offer(int channel, int cc, int data2, CcLearned& out) noexcept
{
    if (isReservedCc(cc))
        return false;
    // The first thread to take the pending parameter wins; others see -1.
    const int idx = pendingParam.exchange(-1, std::memory_order_acq_rel);
    if (idx < 0)
        return false;
    const bool specific = pendingChannelSpecific.load(std::memory_order_relaxed) != 0;
    learned.store(packLearned(cc, specific, channel, data2, idx), std::memory_order_release);
    out.cc = cc;
    out.channel = channel;
    out.paramIndex = idx;
    out.value = static_cast<float>(data2 & 0x7f) / 127.0f;
    return true;
}

void CcMapStore::noteCc(int channel, int cc, int data2) noexcept
{
    last.store(0x80000000U | (static_cast<std::uint32_t>(cc & 0x7f) << 16) |
                   (static_cast<std::uint32_t>(channel & 0xf) << 8) | static_cast<std::uint32_t>(data2 & 0x7f),
               std::memory_order_relaxed);
}

bool CcMapStore::lastCc(int& cc, int& channel, int& value) const noexcept
{
    const std::uint32_t v = last.load(std::memory_order_relaxed);
    if ((v & 0x80000000U) == 0)
        return false;
    cc = static_cast<int>((v >> 16) & 0x7fU);
    channel = static_cast<int>((v >> 8) & 0xfU);
    value = static_cast<int>(v & 0x7fU);
    return true;
}

} // namespace crucible
