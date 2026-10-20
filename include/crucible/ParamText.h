// SPDX-License-Identifier: MIT
#pragma once

// The standard display-text functions of parameters (see TextFormat in ParameterMap.h). A parameter's text format
// turns a natural value into its canonical text and reads text back, including the usual variants ("1.5k",
// "20ms", "C#4", "on"). Floats print with six significant digits, so every value that has at most six significant
// decimal digits survives the round trip exactly.

#include <crucible/ParameterMap.h>

#include <string>

namespace crucible::ParamText
{

// The format a parameter uses: its own when set, else derived from its unit and type.
TextFormat resolveFormat(const ParamDef& def);
// Canonical text of a natural value (clamped to the range; ints, choices and switches are rounded).
std::string format(const ParamDef& def, float value);
// Value of a text, clamped to the range; false when the text cannot be read.
bool parse(const ParamDef& def, const std::string& text, float& out);
// Six-character ASCII label derived from a display name.
std::string shortNameOf(const std::string& name);

} // namespace crucible::ParamText
