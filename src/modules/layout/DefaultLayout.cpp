// SPDX-License-Identifier: MIT
#include "../Modules.h"

// The neutral product layout of the engine: a handful of generic sections, and the pages in the order in which the
// modules declare their parameters. A product that wants a hand-made page order passes its own layout to
// ModuleRegistry::configureProduct(). The layout changes how parameters are grouped and ordered for a UI or a
// hardware surface; it never changes parameter ids, their indices or the sound.

namespace crucible::modules
{

ProductLayout defaultLayout()
{
    ProductLayout l;
    l.id = "crucible.default";
    l.sections = {
        {"osc1", "Oscillators"}, {"osc2", "More oscillators"}, {"env", "Envelopes"}, {"mod", "Modulation"},
        {"fx", "Effects"},
    };
    return l;
}

} // namespace crucible::modules
