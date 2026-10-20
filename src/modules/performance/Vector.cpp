// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/module/ParamBuilder.h>

// The vector field: four scenes at the corners of a square, a puck that blends them. The parameters live here;
// the morph itself runs inside SynthEngine (updateVector), because it has to sit between the host values and the
// parameter smoothers at the control rate. None of these parameters is part of a scene.

namespace crucible::modules
{

ModuleDescriptor vectorPad()
{
    ModuleDescriptor d;
    d.id = "perf.vector";
    d.name = "Vector Field";
    d.kind = ModuleKind::Performance;
    d.category = "Performance";
    d.description = "Four-corner morph between stored scenes with smoothing, orbit and drift.";
    auto& p = d.params;
    const auto add = [&p](ParamDef def)
    {
        def.scene = false;
        p.push_back(std::move(def));
    };
    const std::vector<std::string> slotNames = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};

    add(params::toggle("vector_on", "Vector On", "vector", false));
    // The puck position can be recorded into clips and driven by routes: both are generic modulation targets.
    add(params::modulatable(
        params::floating("vector_x", "Vector X", "vector", 0.0f, 1.0f, 0.5f, "", 1.0f, Smoothing::None)));
    add(params::modulatable(
        params::floating("vector_y", "Vector Y", "vector", 0.0f, 1.0f, 0.5f, "", 1.0f, Smoothing::None)));
    d.params[d.params.size() - 2].uiHint = d.params.back().uiHint = UiHint::Xy;
    add(params::floating("vector_smooth", "Vector Smooth", "vector", 0.0f, 2.0f, 0.05f, "s", 0.5f, Smoothing::None));
    add(params::floating("vector_drift", "Vector Drift", "vector", 0.0f, 0.5f, 0.0f, "", 1.0f, Smoothing::None));
    add(params::floating("vector_drift_rate", "Vector Drift Rate", "vector", 0.02f, 4.0f, 0.2f, "Hz", 0.4f,
                         Smoothing::None));

    add(params::floating("vector_orbit_rate", "Vector Orbit Rate", "vector_2", 0.02f, 10.0f, 0.5f, "Hz", 0.4f,
                         Smoothing::None));
    add(params::floating("vector_orbit_depth", "Vector Orbit Depth", "vector_2", 0.0f, 0.5f, 0.0f, "", 1.0f,
                         Smoothing::None));
    add(params::choice("vector_orbit_shape", "Vector Orbit Shape", "vector_2",
                       {"Circle", "Figure Eight", "Horizontal", "Vertical"}, 0));

    add(params::choice("vector_a", "Vector Corner A", "vector_corners", slotNames, 0));
    add(params::choice("vector_b", "Vector Corner B", "vector_corners", slotNames, 1));
    add(params::choice("vector_c", "Vector Corner C", "vector_corners", slotNames, 2));
    add(params::choice("vector_d", "Vector Corner D", "vector_corners", slotNames, 3));

    d.pages.push_back(
        {"vector", "Vector", "mod", "Vector Field", "LCD shows the square with the puck; strips set the puck position."});
    d.pages.push_back({"vector_2", "Vector Orbit", "mod", "Vector Orbit",
                       "LCD shows the orbit path around the puck; strips set rate and depth."});
    d.pages.push_back({"vector_corners", "Vector Corners", "mod", "Vector Corner Scenes",
                       "LCD shows the scene slots at the four corners A to D."});
    return d;
}

} // namespace crucible::modules
