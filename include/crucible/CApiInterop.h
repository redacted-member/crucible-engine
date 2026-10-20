// SPDX-License-Identifier: MIT
#pragma once

// C++ side doors into the C API handles, for hosts that extend the engine with their own pieces and still want to
// hand them out through the C functions.

#include <crucible/KeyBus.h>
#include <crucible/crucible.h>

#include <memory>

// Wraps a bus in a C handle (close it with crucible_keybus_close). Null in, null out.
CrucibleKeyBus* crucible_keybus_wrap(std::shared_ptr<crucible::KeyBus> bus);
