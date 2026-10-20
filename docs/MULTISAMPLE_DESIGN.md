# Multisample zones for the sample oscillator: design and implementation

Status: implemented (`include/crucible/Zones.h`, `src/Zones.cpp`, the zone path in `src/modules/oscillator/SampleOscillator.cpp`,
`tests/ZoneTests.cpp`); the deviations from this text are listed under "As built" at the end. It touches the sample bank, the patch asset format, the voice's per-chunk sample pinning and the
oscillator's note start, so it is a work package of its own, not a leftover. Today an oscillator slot owns exactly one
`SampleData` (`SampleBank`, 8 slots), pinned for the whole render call (`params.sample[slot]`) and read through
`OscRender::sample`; slices (`smp_slice_*`) already pick a region of that one sample per note.

## Goal

One sample oscillator slot plays a set of zones. A zone is a sample with a key range, a velocity range, a root note, a
gain and optional loop points. A note starts the zone(s) whose ranges contain its key and velocity.

## Data model

* `ZoneSet` (immutable, published per slot through `RtShared`, like `SliceSet`): up to `kMaxZones = 64` zones, each
  `{ int sampleId; uint8 keyLo, keyHi, velLo, velHi; float root; float gainDb; float tuneCents; float loopStart, loopEnd; int loopMode (-1 = use slot parameters); uint8 group; }`.
  `sampleId` indexes the slot's sample list, so several zones can share audio.
* `SampleBank` stores a list of `SampleData` per slot (`kMaxSamplesPerSlot = 64`, total memory capped like
  `kSampleMaxFrames` per slot x 4) next to the current single sample. Entry 0 is today's sample: a slot without a `ZoneSet`
  behaves exactly as now (bit-identical renders, no new parameters read).
* Pinning: `SampleBank::acquire` pins the sample list and zone set per render call (arrays of pointers sized at prepare,
  no allocation). `OscRender` gains `const ZoneSet* zones` and `const SampleData* const* samples`.
* Parameters (appended to the sample oscillator, registered late like `cloudBands`, so generic destination numbers do not
  move): `smp_zone_mode` (Off / Zones), `smp_zone_xfade_key` (0..12 semitones crossfade at zone edges),
  `smp_zone_xfade_vel` (0..0.5), `smp_zone_rr` (Off / Cycle / Random round robin among zones that overlap and share a group).
  Per-zone values are content, not parameters (they are in the ZoneSet, saved with the patch).

## Playing

* `noteOn` (control rate, audio thread): with `smp_zone_mode` = Zones, select up to 4 matching zones into a fixed array
  `active[4]` (zone index, weight). Lowest zone index first; overlapping zones inside one `group` are alternatives
  (round robin by the oscillator's own counter, seeded by `OscNoteInfo::seed` so renders stay reproducible), different groups
  layer. Edge crossfades give weights by distance into the overlap (equal power). No match: the note is silent
  (`finished = true`), as an out-of-range slice is today.
* `render`: each active zone runs the existing `renderSample` path with its own `SampleData`, root (replaces `smp_root`,
  or the slice root), gain and loop points; the output of the layers is summed. Unison, filters and the rest are shared:
  the per-zone state is only the playhead set (`vpos`, `vdir`, `vdone`, 8 unison heads), so the instance holds
  `std::array<ZoneVoice, 4>` allocated with the oscillator (no allocation in `render`).
* Slices stay per sample (`SliceSet` per sample id) and are ignored for zoned slots in version 1.
* Granular mode reads the first active zone only in version 1.
* The sample view for UIs (`SynthEngine::SampleView`) reports the zone list and, per voice, the active zone so a keyboard map
  can light zones.

## Persistence

* Patch: assets of kind `"sample"` already carry `slot`; add `index` (default 0) so several samples per slot travel, and one
  asset of kind `"zones"` per slot (text: JSON array of zones). Patches without `index` / `"zones"` load as today; the
  format version stays 2 (additive fields). Content hashes of old presets are unchanged because the new fields are written only
  for zoned slots. `SampleBank::Record` gets `index`; `serialise` / `deserialise` / `PresetLibrary` asset store follow.
* Import helpers on the control thread (not the audio thread): a folder of WAVs named `Name_C3_v80.wav` or a SFZ-lite
  text (`key`, `lokey`, `hikey`, `lovel`, `hivel`, `pitch_keycenter`, `loop_start`, `loop_end`) builds the ZoneSet; automatic
  key ranges from the detected root (`SampleData::detectedNote`) split the distance between neighbouring roots.

## Tests to write with it

Zone selection by key and velocity; edge crossfade is equal power (sum of squares 1); round robin sequence is deterministic;
a slot without zones renders bit-identically to the current engine; two layers equal the sum of two single-zone renders;
note-off and retrigger fades unchanged; patch round trip with 3 samples and old patch loads; no allocation in `render`
(`PlatformTests` allocation test); zone count and memory caps; sanitizer run.

## Cost and risk

Memory is the main risk (each sample keeps band-limited copies, about 2.5x the audio); cap per slot. CPU: up to 4 layers
multiplies the sample oscillator's cost by the layer count; layers beyond 2 are rare. Crossfades only run where zones overlap.

## As built

* **Storage.** The zone content lives in `ZoneBank` (`SynthEngine::zoneBank()`, `Engine::zones()`) next to `SampleBank`, which is
  unchanged. Sample id 0 is the slot's own `SampleBank` sample (today's sample, so a slot keeps working with its existing
  load / slice API); ids 1..63 are extra samples added with `ZoneBank::addSample`. A published `ZoneSet` holds the zone list
  and shared pointers to the extra samples, so one hazard-pointer pin per slot covers everything and republishing zones does
  not copy audio. Caps: 64 zones, 63 extra samples, `kSampleMaxFrames * 4` frames of extra audio per slot.
* **Selection time.** `OscillatorInstance::noteOn` has no access to the pinned data, so the zones are chosen at the first
  render of the note (the same moment slices are chosen). The key is the rounded note and the velocity is `round(v * 127)`.
* **Crossfades.** `smp_zone_xfade_key` (semitones) and `smp_zone_xfade_vel` (0..0.5 of the range) centre a fade of that width on
  the half-way point between the last key (velocity) of a zone and the first of the next, so two zones that meet at an edge
  fade with sin / cos weights (sum of squares 1). A zone fades out up to half the width beyond its own range; the key and
  velocity weights multiply. Edges at 0 and 127 are open. A lone zone is attenuated by at most a few percent within half the
  width of its edge.
* **Round robin.** Candidates of one group whose ranges overlap are alternatives; one sounds per note (Cycle: a counter on the
  published `ZoneSet`, restarting with every publish; Random: hashed from `OscNoteInfo::seed`). With round robin Off every
  overlapping zone layers. Crossfade neighbours (ranges that do not overlap) are never alternatives of each other.
* **Zone values.** `loopMode >= 0` makes the zone use its own loop mode and `loopStart` / `loopEnd` (0..1 over the played
  region); `smp_start` / `smp_end`, reverse, unison and `smp_key` still apply. With keytracking off the zone plays at the
  original pitch (plus its tuning). Granular mode reads the first selected zone.
* **Parameters.** `smp_zone_mode`, `smp_zone_xfade_key`, `smp_zone_xfade_vel`, `smp_zone_rr` on a new "Zones" page per slot,
  registered after the slice parameters. The module version of `osc.sample` is 2.
* **Persistence.** Assets of kind `"sample"` gained `index` (written only when not 0, so old files and plain slots are
  byte-identical) and kind `"zones"` (JSON text, one asset per zoned slot); `PatchAsset` / `PatchAssetRef` got `index` as their
  last member. The patch format version stays 2. Applying a patch clears the zone content of every slot first.
* **Import.** `ZoneBank::importFolder` (files named `Name_C3_v80.wav`; C3 = 60, `_60` is a plain root, no root = detected pitch,
  ranges from `assignAutomaticRanges`) and `importSfz` (`<group>` / `<region>`, `sample`, `key`, `lokey`, `hikey`, `lovel`,
  `hivel`, `pitch_keycenter`, `loop_start`, `loop_end`, `loop_mode`, `volume`, `tune`, `transpose`).
* **Interfaces.** C API: `crucible_add_zone_sample`, `crucible_set_zones`, `crucible_get_zones`, `crucible_zone_sample_count`,
  `crucible_clear_zones`, `crucible_import_zone_folder`, `crucible_import_zone_sfz` (`CrucibleZone`). Facade: `Engine::addZoneSample`,
  `setZones`, `zoneList`, `zoneSampleCount`, `clearZones`, `importZoneFolder`, `importZoneSfz`. `SampleView` gained `zones`, and
  `OscVisual` gained `zones` (the zone indices sounding in the active voices).
