// SPDX-License-Identifier: MIT
#pragma once

// KeyBus: one shared key / scale for every engine that joins it.
//
// The core knows two buses: a private one (createPrivate(), for tests) and the process-wide one (local(), shared by
// every engine in the process). A host that needs the key to cross process boundaries derives from KeyBus and points
// it at memory it maps itself (attachCell); the product adapters of this repository do that with a small mmap'd file.
// Reading on the audio thread is one atomic load, writing is one compare-exchange, so neither side blocks, allocates
// or makes a system call.
//
// Cell layout (little endian, two 64-bit atomics):
//   state   bits 0..11   scale mask (intervals from the root, bit 0 = root)
//           bits 12..15  root pitch class
//           bit  16      tie-break up
//           bits 32..63  generation, +1 for every publish (never 0 once something was published)
//   writer  id of the last writer (informational)
//
// Semantics. Last writer wins: publish() installs the new key and bumps the generation whatever it was before.
// A follower adopts the state whenever the generation differs from the one it used last. Leaders publish when
// their own key changes. Generation 0 means "nothing published yet": followers keep their local key.

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace crucible
{

struct KeyBusState
{
    int root = 0;
    std::uint16_t mask = 0x0AB5;
    bool tieUp = false;
    std::uint32_t generation = 0;
    std::uint64_t writer = 0;
};

// The two words a bus is made of. Shared-memory transports place one of these inside their mapped block.
struct KeyBusCell
{
    std::atomic<std::uint64_t> state;
    std::atomic<std::uint64_t> writer;
};
static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "the bus needs lock-free 64-bit atomics");

class KeyBus
{
public:
    virtual ~KeyBus() = default;
    KeyBus(const KeyBus&) = delete;
    KeyBus& operator=(const KeyBus&) = delete;

    // Private memory, shared by every user of local() in this process.
    static std::shared_ptr<KeyBus> local();
    // A fresh private bus (tests).
    static std::shared_ptr<KeyBus> createPrivate();

    // Audio-thread safe.
    KeyBusState read() const noexcept;
    // Returns the new generation.
    std::uint32_t publish(int root, std::uint16_t mask, bool tieUp, std::uint64_t writerId = 0) noexcept;

    // True when the cell lives in memory shared with other processes.
    virtual bool isShared() const { return false; }
    virtual const std::string& path() const;

protected:
    KeyBus();
    // Redirects the bus to a cell the subclass owns (for example inside a mapped file). The cell has to stay valid
    // for the lifetime of the bus.
    void attachCell(KeyBusCell* cell) noexcept { active = cell; }

private:
    KeyBusCell ownCell;
    KeyBusCell* active;
};

} // namespace crucible
