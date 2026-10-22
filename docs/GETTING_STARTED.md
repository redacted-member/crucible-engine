<!-- SPDX-License-Identifier: MIT -->
# Getting started: a CLAP or VST3 instrument on Crucible

This page is for someone writing the plug-in wrapper. Crucible makes the sound; your wrapper owns the host
conversation, the GUI and the state chunk. The whole lifecycle is in one small C file,
[`examples/instrument_skeleton/main.c`](../examples/instrument_skeleton/main.c), with a fake host in place of CLAP /
VST3. It runs as the ctest `instrument_skeleton`. Copy it and its product JSON to start. Every call below is declared in
[`include/crucible/crucible.h`](../include/crucible/crucible.h).

## Lifecycle

1. Instantiate [main]: `crucible_config_init`, then `crucible_create_from_product(config, json, NULL, error, size)`.
   On failure it returns NULL and the error text names the problem. Build your host parameter list here (below).
2. Activate [main]: `crucible_prepare(engine, sample_rate, max_block_size)`. Subscribe views here.
3. Per block [audio]: `crucible_process_events(engine, left, right, n, midi, n_midi, params, n_params, transport)`.
   It never allocates or locks. Fill `CrucibleTransport.struct_size` with `sizeof` (0, or a NULL transport, means "no
   transport"). Parameter events must be sorted by position; a position is clamped to the block.
4. Regularly [main]: `crucible_service(engine)`, from the host idle callback or a timer. A changed oscillator type,
   wavetable or sequencer pattern that arrived through `crucible_process_events` takes effect after this call.
5. Destroy [main]: unsubscribe views, then `crucible_destroy`.

| Thread | Calls |
| --- | --- |
| audio | `crucible_process`, `crucible_process_host`, `crucible_process_aux`, `crucible_process_events`, `crucible_process_events_aux`, `crucible_reset` (between blocks) |
| main | `crucible_set_param*`, `crucible_midi`, `crucible_service`, `crucible_param_changes`, `crucible_latency_samples`, patches, `crucible_load_ir`, wavetable loads, view find / subscribe, `crucible_view_read` of a pulled view |
| any | `crucible_set_render_offline`, `crucible_note_on` / `_note_off` / `_pitch_bend` / `_mod_wheel` / `_sustain` / `_all_notes_off` (one thread at a time), `crucible_get_param`, `crucible_modulated_value`, `crucible_meter`, `crucible_tail_seconds`, `crucible_service_pending`, parameter metadata, `crucible_view_read` of a pushed (live) view |

In a plug-in, host automation and note events reach the engine on the audio thread, so turn them into
`CrucibleParamEvent` and `CrucibleMidiEvent` and pass them to `crucible_process_events` (with a sidechain bus,
`crucible_process_events_aux`, which also takes the aux buffers). Do not call
`crucible_set_param` from the audio callback. When the host renders offline (CLAP render mode, VST3 `kOffline`), call
`crucible_set_render_offline(engine, 1)` so `fx.convolution` and `fx.cabinet` wait for their background builds and the
render does not depend on thread timing; set it back to 0 for real time. `crucible_service_pending(engine)` (any thread) says whether a `crucible_service` call has work to do, so a wrapper can ask its host for a main-thread callback; an offline bounce calls `crucible_service` between blocks.

## Host parameters

The product's `"surface"` lists the parameters a host should show (`"expose": "listed"`), grouped into pages. The
others still load from patches and can be modulation targets, but a host never sees them.

- Walk `crucible_surface_count` / `crucible_surface_param(engine, k)` to get engine indexes in page order.
- `crucible_param_meta(engine, index, &meta)` (set `meta.struct_size = sizeof meta` first; the same for `CrucibleModuleMeta` and
  `CrucibleViewInfo`) gives `name`, `min`, `max`, `default_value`, `page_id` and `host_id`.
- `host_id` is a 31-bit hash of the parameter id (never in the VST3 reserved range). Use it as the CLAP `param_id` or VST3 `ParamID`; it does not change
  when parameters are added. Resolve an incoming id with `crucible_param_index_by_host_id` (a table lookup, safe per
  host event; -1 if unknown).
- Values are natural units (Hz, dB, choice number). `crucible_value_to_text` and `crucible_text_to_value` do the host's
  value strings; `crucible_set_param_text(engine, "osc1_type", "analog")` sets choices by name.
- Choice numbers are the same in every product. `crucible_choice_available` says whether this product includes one.

## State

Save and restore with the patch, which is the state chunk.

- Save [main]: `int n = crucible_save_patch(engine, "name", buf, cap)` writes into your buffer and returns the full
  length (like `snprintf`); call it with `buf = NULL` to learn the size first. `crucible_describe_json` works the same way.
- Load [main]: `crucible_load_patch(engine, json, error, size)`; 1 on success.

Patches are versioned JSON. See "State serialisation" in the [README](../README.md#state-serialisation) for what loads
and what is ignored.

## Impulse responses

`crucible_load_ir(engine, "fx.convolution", path, error, size)` loads a WAV or AIFF into one convolution instance, and
`crucible_clear_ir` removes it. Each `fx.convolution` instance in the product has its own IR; a second one is
`fx.convolution.2`. A saved patch embeds the IR of every `fx.convolution` instance (`convolutionIrs`, keyed by instance id, base64); version 2 patches with a single `convolutionIr` load into the first instance.

## Choose the voice

For a polysynth use `"voice": "classic"` and restrict the surface to the parameters the instrument needs. Graph
voices are for drums, layered and one-shot instruments, until per-voice node modulation lands: graph node parameters
cannot be modulated per voice yet, so there is no velocity to cutoff or envelope to pitch through routes.

## Start a product

- [`docs/products/template.json`](products/template.json): a classic voice, 2 oscillators and a short chain of current
  modules. Copy it, change the id, add or remove instances, and list the surface.
- The README table "Which module do I use" ([README](../README.md#which-module-do-i-use)) shows what replaced the removed
  first-generation effects. Use these modules in a new product.
- [`examples/instrument_skeleton/skeleton_product.json`](../examples/instrument_skeleton/skeleton_product.json) is the
  template with a 15-parameter surface on pages.
- [`examples/instrument_skeleton/skeleton_full_product.json`](../examples/instrument_skeleton/skeleton_full_product.json) is
  the same path with every current module (classic voice, 3 oscillator slots, 2 filters, 6 macros and all 29 effects in one
  chain). `tests/SkeletonFullTests.cpp` (`ctest -R skeleton_all_features`) renders it through the C API with every
  oscillator type, filter model (Formant with a user table and saturation), effect, user wavetable zones, a shaper curve, the
  sequencer, arpeggiator, macros and routes, and checks finite, bounded, audible, deterministic and a patch round trip.

## Engine-made changes, latency and reset

- Every block you hand `crucible_process_events` host parameter events. Changes the engine makes itself (a patch load, a
  mapped MIDI CC, a scene recall or morph, sequencer coupling) are collected for you: from [main], in the idle callback or a
  timer, call `crucible_param_changes(engine, indices, sources, cap)` until it returns 0. Skip entries whose `sources[k]` is
  `CRUCIBLE_SOURCE_HOST` (the host already knows) and send the rest to the host. An entry with index `CRUCIBLE_CHANGE_LATENCY`
  (-1) means the latency changed: ask `crucible_latency_samples(engine)` and tell the host. `CRUCIBLE_CHANGE_TAIL` (-2) means
  `crucible_tail_seconds(engine)` changed.
- At activate, report `crucible_latency_samples(engine)`, and `crucible_tail_seconds(engine)` for the host's tail query
  (`HUGE_VAL` means it never decays).
- `crucible_reset(engine)` [audio: between process calls, never concurrently] allocates and locks nothing; it clears voices,
  held notes and every effect's state without re-preparing. Call it on activate, on a transport jump and when the host resets or bypasses the plug-in.

## Views for a GUI

`crucible_view_find(engine, "master", "meter")` (3 elements: peak left dB, peak right dB, clip) and
`"params"` / `"<id>.mod"` (the value after modulation, 1 element) are live views: subscribe with `crucible_view_subscribe`.
`"filter"` / `"response"` (and `"filter2"`, ... for more filters; `"response_mod"` with modulation applied) is a 256-point
magnitude curve in dB that the reader computes when asked; such pulled views (`live == 0` in `crucible_view_info`) are read
from [main] only, pushed ones from any thread. `crucible_view_size(engine, view)` gives the number of elements to
size your read buffer with; `crucible_view_info` lists all views, and `crucible_describe_json` has them under `views`.

## Modulation and wavetables

- `crucible_add_route(engine, source, "filter_cutoff", amount)` adds an assignable route; `crucible_add_route_ex` with a
  `CrucibleRoute` also sets polarity, curve and smoothing.
- `crucible_load_wavetable_file(engine, slot, path, &info, error, size)` and `crucible_load_wavetable(engine, slot, frames,
  frame_count, frame_size, name, &info, error, size)` fill one of `CRUCIBLE_USER_WAVETABLES` slots; `info` reports whether
  frames were dropped (`frames_in`, `frames_used`, `decimated`); set `info.struct_size = sizeof info` first.
  `crucible_load_wavetable_zones` takes 1 to 16 `CrucibleWavetableZone` (frames plus ascending `max_hz`) so the source can change with pitch.

### Wavetable display and mod matrix

Draw a wavetable with the pulled view `oscN/wavetable`: its first element is the frame count, then 64 frames of 256 points,
frame-major, for the table the slot selects (factory or user; empty for an empty user slot). Size the buffer with
`crucible_view_size`. Spectral tables are not covered by this view yet. Draw the playing position from `params/oscN_pos.mod`
(or the `.voice` / `.band` views), which is the modulated frame position. For the mod matrix, `crucible_route_count` and
`crucible_route_get(k)` read the routes in creation order. Key a route by (source, target), not by `k`: removing a route shifts
the ones after it. Re-read the list when `crucible_param_changes` reports `CRUCIBLE_CHANGE_ROUTES` (-3).

## What a wrapper still does itself

- Copy the patch into the host's state chunk, and send host automation and notes through `crucible_process_events` (never
  `crucible_set_param` from the audio callback).
- Tell the host about parameter changes and latency changes from the lists above. The engine collects them, it does not call
  the host.
- Draw the GUI. Views give you the numbers; `crucible_value_to_text` returns a bare number for some parameters, so add the
  unit if you want one.
- Voices of the graph kind cannot be modulated per voice node yet (see "Choose the voice").
