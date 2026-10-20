// SPDX-License-Identifier: MIT
#pragma once

// Motion recorder: records live parameter moves into a clip lane (see Clips.h), independent of the DAW.
//
// The audio thread samples the smoothed value of one parameter at every 32-sample control chunk, timestamps it in
// clip units and pushes the samples that belong to a touch (the value is moving) into a lock-free ring. The
// control thread drains the ring, thins the samples, optionally snaps them to the clip grid and merges them into
// the lane (overdub) or replaces the lane on the first pass.
//
// Recorded values are stored relative to the parameter's value when recording was armed (lane value 0.5 = no
// offset, 0 / 1 = a full range down / up) and the lane is routed to the parameter bipolar with amount 1. Playback
// therefore reproduces the recorded move on top of the parameter, leaves host automation of the parameter intact
// and works for every kind of route destination.
//
// Time base: a clip that is running on a global playhead (Transport or GlobalFree trigger) is recorded at the
// position of that playhead, so a take lines up with what is playing and overdubs are sample-accurate relative
// to the transport. Otherwise the recorder uses its own clock that starts at arm() and wraps at the clip length
// (or, with detectLength, runs until stop() and sets the clip length from the take).

#include <crucible/Clips.h>
#include <crucible/Modulation.h>
#include <crucible/ParameterMap.h>
#include <crucible/SpscRing.h>

#include <array>
#include <atomic>
#include <string>
#include <vector>

namespace crucible
{

struct MotionSettings
{
    int clip = 0;
    int lane = 0;
    std::string param; // parameter id
    // Optional second take recorded at the same time into a second lane of the same clip (a vector path is
    // vector_x into lane and vector_y into lane2). Empty = a single parameter.
    std::string param2;
    int lane2 = 1;
    bool overdub = false;      // false: the lane is cleared when recording is armed
    bool detectLength = false; // set the clip length from the take (rounded to the clip grid, or a beat)
    bool quantise = false;     // snap recorded points to the clip grid
    float tolerance = 0.004f;  // thinning tolerance (fraction of the lane range)
};

struct MotionSample
{
    double time = 0.0;      // clip units
    std::uint8_t track = 0; // 0: param / lane, 1: param2 / lane2
    float value = 0.0f;     // parameter, normalised 0..1
    bool touched = true;    // false closes a touch
};

class MotionRecorder
{
public:
    MotionRecorder(ClipStore& clips, ModRouteStore& routes);

    // ---- Control thread.
    // Creates the clip / lane when missing (a four-beat looping clip on the transport), routes the lane to the
    // parameter when no route exists, and starts recording. baseNormalised: the parameter's current value, 0..1.
    // baseNormalised2: the same for param2 (ignored without one).
    bool arm(const MotionSettings& settings, float baseNormalised, float baseNormalised2 = 0.5f);
    // Commits what was recorded. With detectLength the clip is resized to the take.
    void stop();
    bool armed() const { return active.load(std::memory_order_acquire); }
    // Moves samples from the audio thread into the lane. Call regularly (a UI timer) while armed and once after
    // stop(). Returns the number of points added.
    int drain();
    // Samples lost because the ring was full (control thread was not draining).
    int dropped() const { return lost.load(std::memory_order_relaxed); }
    // Recording position in clip units (any thread).
    double position() const { return recTime.load(std::memory_order_relaxed); }

    // ---- Audio thread.
    void setSampleRate(double hz) { sampleRate = hz; }
    // values: natural-unit parameter values; headPosition: position of the target clip's global playhead in clip
    // units, or a negative number when it has none. Cheap when not armed.
    void capture(int numSamples, double bpm, const float* values, double headPosition);
    int targetClip() const { return clipIndex.load(std::memory_order_relaxed); }
    bool followsHead() const { return useHead.load(std::memory_order_relaxed); }

private:
    void commitRun(int track);
    void finish();

    ClipStore& clipStore;
    ModRouteStore& routeStore;
    const ParameterModel* model;
    MotionSettings cfg;
    std::array<float, 2> base{};
    std::array<std::vector<MotionSample>, 2> run;
    std::vector<ClipPoint> runPoints;

    SpscRing<MotionSample, 2048> ring;
    std::atomic<bool> active{false};
    std::array<std::atomic<int>, 2> paramIndex{{{-1}, {-1}}};
    std::atomic<int> clipIndex{0};
    std::atomic<bool> useHead{false};
    std::atomic<bool> own{false}; // own clock instead of a playhead
    std::atomic<bool> wrap{true};
    std::atomic<bool> sync{true};
    std::atomic<double> rate{1.0};
    std::atomic<double> length{4.0};
    std::atomic<double> recTime{0.0};
    std::atomic<int> lost{0};
    std::atomic<bool> restartClock{false};

    // Audio thread state.
    bool wasActive = false;
    std::array<bool, 2> touching{};
    std::array<float, 2> lastNorm{{-1.0f, -1.0f}};
    std::array<int, 2> idleChunks{};
    double clock = 0.0;
    double sampleRate = 48000.0;
};

// Rounds a recorded duration to a whole number of grid steps (a beat / a second when the grid is off), at least one.
double detectClipLength(const Clip& clip, double duration);

// Moves the points of a curve lane towards the grid (strength 0..1), merging points that land on the same place.
void quantiseLane(const Clip& clip, ClipLane& lane, float strength = 1.0f);

} // namespace crucible
