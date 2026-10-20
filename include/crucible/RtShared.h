// SPDX-License-Identifier: MIT
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace crucible
{

// Immutable data handed from a control thread to one audio-thread reader without locks or allocation on the
// audio side. The control thread publishes a new object; the reader pins the current one with a hazard pointer
// for the duration of its use. Superseded objects are freed by collectGarbage() (control thread) once the reader
// no longer holds them. One reader per instance.
template <typename T> class RtShared
{
public:
    RtShared() = default;
    RtShared(const RtShared&) = delete;
    RtShared& operator=(const RtShared&) = delete;

    // Control thread. Passing nullptr publishes "nothing".
    void publish(std::unique_ptr<T> object)
    {
        std::lock_guard<std::mutex> lock(mutex);
        T* raw = object.get();
        if (object)
            owned.push_back(std::move(object));
        current.store(raw, std::memory_order_seq_cst); // pairs with the reader's seq_cst pin-and-recheck
        collectLocked();
    }

    void collectGarbage()
    {
        std::lock_guard<std::mutex> lock(mutex);
        collectLocked();
    }

    // Control thread: the object most recently published (may be read while the audio side also reads it).
    const T* latest() const { return current.load(std::memory_order_acquire); }

    // Any thread except the audio thread: runs f on the newest object while publish() and collectGarbage() are held
    // off, so the object cannot be freed under the caller. Returns false (f not called) when nothing is published.
    // f must be short and must not call back into this object.
    template <typename F> bool withLatest(F&& f) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const T* p = current.load(std::memory_order_acquire);
        if (p == nullptr)
            return false;
        f(*p);
        return true;
    }

    // Control thread: the object the reader has pinned right now, or nullptr.
    const T* pinned() const { return inUse.load(std::memory_order_seq_cst); }

    // Audio thread: pin, use, release. Returns nullptr when nothing is published.
    const T* acquire() noexcept
    {
        for (;;)
        {
            T* p = current.load(std::memory_order_seq_cst);
            inUse.store(p, std::memory_order_seq_cst);
            if (current.load(std::memory_order_seq_cst) == p)
                return p;
        }
    }
    void release() noexcept { inUse.store(nullptr, std::memory_order_release); }

private:
    void collectLocked()
    {
        const T* keep1 = current.load(std::memory_order_seq_cst);
        const T* keep2 = inUse.load(std::memory_order_seq_cst);
        std::vector<std::unique_ptr<T>> kept;
        for (auto& o : owned)
            if (o.get() == keep1 || o.get() == keep2)
                kept.push_back(std::move(o));
        owned = std::move(kept);
    }

    mutable std::mutex mutex;
    std::vector<std::unique_ptr<T>> owned;
    std::atomic<T*> current{nullptr};
    std::atomic<T*> inUse{nullptr};
};

} // namespace crucible
