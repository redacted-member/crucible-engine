// SPDX-License-Identifier: MIT
#include <crucible/KeyBus.h>

namespace crucible
{
namespace
{
std::uint64_t pack(int root, std::uint16_t mask, bool tieUp, std::uint32_t generation)
{
    return static_cast<std::uint64_t>(mask & 0x0FFFU) | (static_cast<std::uint64_t>(root & 0xF) << 12) |
           (tieUp ? (1ULL << 16) : 0ULL) | (static_cast<std::uint64_t>(generation) << 32);
}

class PrivateKeyBus final : public KeyBus
{
public:
    PrivateKeyBus() = default;
};
} // namespace

KeyBus::KeyBus() : active(&ownCell)
{
    ownCell.state.store(0);
    ownCell.writer.store(0);
}

const std::string& KeyBus::path() const
{
    static const std::string none;
    return none;
}

std::shared_ptr<KeyBus> KeyBus::createPrivate()
{
    return std::make_shared<PrivateKeyBus>();
}

std::shared_ptr<KeyBus> KeyBus::local()
{
    static std::shared_ptr<KeyBus> bus = createPrivate();
    return bus;
}

KeyBusState KeyBus::read() const noexcept
{
    const std::uint64_t s = active->state.load(std::memory_order_acquire);
    KeyBusState out;
    out.mask = static_cast<std::uint16_t>(s & 0x0FFFU);
    out.root = static_cast<int>((s >> 12) & 0xFU) % 12;
    out.tieUp = ((s >> 16) & 1U) != 0;
    out.generation = static_cast<std::uint32_t>(s >> 32);
    out.writer = active->writer.load(std::memory_order_relaxed);
    return out;
}

std::uint32_t KeyBus::publish(int root, std::uint16_t mask, bool tieUp, std::uint64_t writerId) noexcept
{
    root = ((root % 12) + 12) % 12;
    std::uint64_t old = active->state.load(std::memory_order_acquire);
    std::uint32_t generation;
    do
    {
        generation = static_cast<std::uint32_t>(old >> 32) + 1;
        if (generation == 0)
            generation = 1;
    } while (!active->state.compare_exchange_weak(old, pack(root, mask, tieUp, generation), std::memory_order_acq_rel));
    active->writer.store(writerId, std::memory_order_relaxed);
    return generation;
}

} // namespace crucible
