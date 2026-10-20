// SPDX-License-Identifier: MIT
#pragma once

// Whether the test binaries can count allocations by replacing the global operator new / delete.
// On Linux the static AddressSanitizer / ThreadSanitizer runtimes define operator new / delete themselves, so the
// replacement would be a duplicate symbol at link time. There the replacement is not compiled
// (CRUCIBLE_ALLOC_COUNTER_AVAILABLE is 0), the counter stays at zero and the allocation checks are vacuous; the plain
// Release and scalar builds carry them. Tests that need the counter to see an allocation skip themselves instead.

#if defined(__linux__) && (defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__))
#define CRUCIBLE_ALLOC_COUNTER_AVAILABLE 0
#elif defined(__linux__) && defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define CRUCIBLE_ALLOC_COUNTER_AVAILABLE 0
#endif
#endif
#ifndef CRUCIBLE_ALLOC_COUNTER_AVAILABLE
#define CRUCIBLE_ALLOC_COUNTER_AVAILABLE 1
#endif
