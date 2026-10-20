"""Pending tests of the analog-behaviour analysis types (measure_analog.py).

They are not part of build_tests(): the reference plugins have not been driven through them and the engine does
not implement the behaviours yet, so they are listed only with `refbench.py plan --include-pending` and their
targets in targets.json carry "status": "pending". Role names that no profile defines yet (for example
"filter.selfosc" or "amp_env.mode.analog_rc") make a test skip with the missing role until a profile adds them.
"""

from __future__ import annotations

from .suite import Test, note

DRIFT_NOTES_PER_RENDER = 50
DRIFT_RENDERS = 4  # 200 notes of 10 s in total, split so that no render is larger than about 200 MB
DRIFT_NOTE_SECONDS = 10.0


def build_pending_tests() -> list[Test]:
    t: list[Test] = []

    # Self-oscillation tracking: maximum resonance, oscillator cut at 0.1 s, cutoff swept over the keyboard.
    for fc in (40.0, 100.0, 250.0, 1000.0, 3000.0, 8000.0):
        t.append(Test(
            name=f"selfosc_track_{int(fc)}", group="analog", seconds=3.0, notes=[note(60, 0.0, 2.9)],
            roles=["saw", "filter.enable", "filter.types.ladder", "filter.cutoff", "filter.resonance"],
            values={"filter.cutoff": fc, "filter.resonance": 1.0},
            ramps=[{"role": "osc_kill", "start": 0.1, "end": 0.11}],
            analysis={"type": "selfosc_track", "cutoff": fc, "window": [1.2, 2.9]},
            description=f"ladder, resonance 1, cutoff {fc:g} Hz: pitch of the ringing against the setting"))

    # Bass loss: a 46 Hz sine at -40 dBFS through a 1 kHz ladder at rising resonance, against the open filter.
    bass_note = 30
    t.append(Test(
        name="bassloss_ref", group="analog", seconds=2.0, notes=[note(bass_note, 0.0, 1.9)],
        roles=["sine", "osc.level_db", "filter.open"], values={"osc.level_db": 0.01},
        analysis={"type": "res_bassloss", "note": bass_note, "window": [0.5, 1.9]},
        description="reference for the bass-loss tests: -40 dBFS sine at 46 Hz, filter open"))
    for res in (0.0, 0.25, 0.5, 0.75, 0.95):
        t.append(Test(
            name=f"bassloss_r{int(res * 100)}", group="analog", seconds=2.0, notes=[note(bass_note, 0.0, 1.9)],
            roles=["sine", "osc.level_db", "filter.enable", "filter.types.ladder", "filter.cutoff", "filter.resonance"],
            values={"osc.level_db": 0.01, "filter.cutoff": 1000.0, "filter.resonance": res},
            analysis={"type": "res_bassloss", "ref": "bassloss_ref", "resonance": res, "note": bass_note,
                      "window": [0.5, 1.9]},
            description=f"ladder at 1 kHz, resonance {res:g}: gain of a 46 Hz sine against the open filter"))

    # Level dependence: gain at the resonant peak (cutoff 1 kHz, resonance 0.9) for four input levels.
    for level in (-40.0, -12.0, 0.0, 6.0):
        lin = 10 ** (level / 20.0)
        base = {"osc.level_db": lin, "filter.cutoff": 1000.0, "filter.resonance": 0.9}
        t.append(Test(
            name=f"resdep_ref_l{int(level)}", group="analog", seconds=2.0, notes=[note(83, 0.0, 1.9)],
            roles=["sine", "osc.level_db", "filter.open"], values={"osc.level_db": lin},
            analysis={"type": "res_level_dependence", "level_db": level, "freq_hz": 987.77, "note": 83,
                      "window": [0.5, 1.9]},
            description=f"reference for the level-dependence tests at {level:g} dBFS"))
        t.append(Test(
            name=f"resdep_l{int(level)}", group="analog", seconds=2.0, notes=[note(83, 0.0, 1.9)],
            roles=["sine", "osc.level_db", "filter.enable", "filter.types.ladder", "filter.cutoff", "filter.resonance"],
            values=base,
            analysis={"type": "res_level_dependence", "ref": f"resdep_ref_l{int(level)}", "level_db": level,
                      "freq_hz": 987.77, "note": 83, "window": [0.5, 1.9]},
            description=f"ladder at 1 kHz, resonance 0.9, sine at the peak, {level:g} dBFS in"))

    # Saturation: harmonics of a 1 kHz and a 50 Hz sine at stepped drive.
    for label, n in (("1k", 83), ("50", 31)):
        for drive in (0.0, 0.25, 0.5, 0.75, 1.0):
            t.append(Test(
                name=f"sat_{label}_d{int(drive * 100)}", group="analog", seconds=1.6, notes=[note(n, 0.0, 1.5)],
                roles=["sine", "fx.drive.enable", "fx.drive.amount"], values={"fx.drive.amount": drive},
                analysis={"type": "sat_harmonics", "note": n, "drive": drive, "window": [0.4, 1.4], "harmonics": 12},
                description=f"sine at note {n} through the saturator at drive {drive:g}"))

    # Drift: many notes of 10 s, pitch tracked every 50 ms.
    for part in range(DRIFT_RENDERS):
        starts = [i * DRIFT_NOTE_SECONDS for i in range(DRIFT_NOTES_PER_RENDER)]
        t.append(Test(
            name=f"drift_stats_{part}", group="analog", seconds=DRIFT_NOTES_PER_RENDER * DRIFT_NOTE_SECONDS,
            notes=[note(69, s, DRIFT_NOTE_SECONDS - 0.3) for s in starts],
            roles=["sine", "drift.amount"], values={"drift.amount": 1.0},
            analysis={"type": "drift_stats", "note": 69, "starts": starts, "length": DRIFT_NOTE_SECONDS - 0.2,
                      "skip": 0.5, "frame": 4096, "hop": 2400, "slope_band": [0.3, 3.0],
                      "nominal_std": 6.0, "nominal_tau": 2.0},
            description=f"{DRIFT_NOTES_PER_RENDER} notes of 10 s (part {part + 1} of {DRIFT_RENDERS}), "
                        "drift at maximum: pitch statistics against the programmed Ornstein-Uhlenbeck process"))

    # Analog RC envelope and retrigger.
    env = {"amp_env.attack": 0.1, "amp_env.decay": 0.3, "amp_env.sustain": 0.5, "amp_env.release": 0.4}
    t.append(Test(
        name="env_rc", group="analog", seconds=2.4, notes=[note(69, 0.1, 1.2)],
        roles=["sine", "amp_env.attack", "amp_env.decay", "amp_env.sustain", "amp_env.release", "amp_env.mode.analog_rc"],
        values=env,
        analysis={"type": "env_rc_fit", "note": 69, "note_on": 0.1, "note_off": 1.3, "attack": 0.1, "release": 0.4},
        description="analog RC envelope: attack 100 ms, decay 300 ms, sustain 50 %, release 400 ms"))
    t.append(Test(
        name="env_rc_retrigger", group="analog", seconds=2.4, notes=[note(69, 0.1, 0.5), note(69, 0.7, 0.8)],
        roles=["sine", "amp_env.attack", "amp_env.decay", "amp_env.sustain", "amp_env.release", "amp_env.mode.analog_rc"],
        values=env,
        analysis={"type": "env_rc_fit", "note": 69, "note_on": 0.1, "note_off": 0.6, "attack": 0.1, "release": 0.4,
                  "retrigger": 0.7},
        description="analog RC envelope retriggered during its release: the level must continue, not jump"))

    # Voice cards: 64 notes of one pitch, pitch and level per note.
    t.append(Test(
        name="voice_variance", group="analog", seconds=64 * 1.2, notes=[note(60, i * 1.2, 1.0) for i in range(64)],
        roles=["saw", "voice_card.amount"], values={"voice_card.amount": 1.0},
        analysis={"type": "voice_variance", "note": 60, "notes": 64, "period": 1.2, "window": [0.3, 0.95]},
        description="64 notes of C4, voice-card tolerance at maximum: spread of pitch and level between voices"))
    return t
