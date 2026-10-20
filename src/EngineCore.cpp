// SPDX-License-Identifier: MIT
#include <crucible/EngineCore.h>
#include <crucible/Denormals.h>

#include <crucible/ParameterMap.h>

#include "EngineViews.h"

#include <algorithm>
#include <cassert>

namespace crucible
{
EngineCore::EngineCore(const EngineConfig& config) : synth(config)
{
    const ParameterModel& model = synth.model();
    const auto need = [&model](const char* id)
    {
        const int i = model.indexOf(id);
        assert(i >= 0);
        return std::max(i, 0);
    };
    idx = {need("seq_run"),    need("seq_sync"),      need("seq_tempo"),       need("seq_rate"),     need("seq_dir"),
           need("seq_swing"),  need("seq_transpose"), need("seq_pattern"),     need("seq_chain"),    need("seq_rec"),
           need("meter_hold"), need("arp_on"),        need("arp_mode"),        need("arp_rate"),     need("arp_octaves"),
           need("arp_gate"),   need("arp_latch"),     need("arp_sync"),        need("arp_tempo"),    need("arp_free_rate"),
           need("arp_swing"),  need("arp_ratchet"),   need("arp_transpose"),   need("arp_vel_mode"), need("arp_vel"),
           need("arp_accent"), need("arp_grid"),      need("arp_grid_length"), need("arp_grid_dir"), need("mpe_on"),
           need("key_mode"),   need("key_root"),      need("key_scale"),       need("key_mask"),     need("key_tie"),
           need("key_sync")};
    writerId = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
    const auto n = model.params().size();
    working.assign(n, 0.0f);
    locked.assign(n, 0.0f);
    lockActive.assign(n, 0);
    exclusions.assign(n, 0);
    synth.setVectorExclusions(exclusions.data());
    EngineViews::install(*this);
}

void EngineCore::refreshExclusions(size_t index)
{
    exclusions[index] = (lockActive[index] != 0 || (productExclude != nullptr && productExclude[index] != 0)) ? 1 : 0;
}

void EngineCore::setProductExclusions(const char* mask)
{
    productExclude = mask;
    for (size_t i = 0; i < exclusions.size(); ++i)
        refreshExclusions(i);
}

void EngineCore::prepare(double sampleRate)
{
    rate = sampleRate;
    quantiser.reset();
    arpQuantiser.reset();
    soundingNow = {};
    soundingTouched = {};
    haveLead = false;
    synth.prepare(sampleRate);
    seq.prepare(sampleRate);
    outputMeter.prepare(sampleRate);
    masterAnalyzer.prepare(sampleRate);
    viewHub.prepare(sampleRate);
    std::fill(lockActive.begin(), lockActive.end(), static_cast<char>(0));
    for (size_t i = 0; i < exclusions.size(); ++i)
        refreshExclusions(i);
    externalRing.reset();
    recordRing.reset();
    seqWasActive = false;
    arp.prepare(sampleRate);
    arpWasOn = false;
}

void EngineCore::reset()
{
    quantiser.reset();
    arpQuantiser.reset();
    soundingNow = {};
    soundingTouched = {};
    haveLead = false;
    synth.reset();
    arp.reset();
    // The note calls own the producer end of the ring, so drain from the consumer side instead of rewinding it. The
    // record ring's consumer is the control thread, which popRecord() serves; it is left alone.
    ExternalEvent discarded;
    while (externalRing.pop(discarded))
    {
    }
}

bool EngineCore::pushExternal(const ExternalEvent& e)
{
    return externalRing.push(e);
}

bool EngineCore::popRecord(RecordEvent& e)
{
    return recordRing.pop(e);
}

int EngineCore::priority(Event::Kind k)
{
    switch (k)
    {
    case Event::Kind::NoteOff:
    case Event::Kind::SeqNoteOff:
        return 0;
    case Event::Kind::LockClear:
        return 1;
    case Event::Kind::LockSet:
        return 2;
    case Event::Kind::Position:
        return 3;
    case Event::Kind::Controller:
    case Event::Kind::Slide:
        return 4;
    case Event::Kind::NoteOn:
    case Event::Kind::SeqNoteOn:
        return 5;
    case Event::Kind::PolyPressure:
        return 6; // after the note-on of its key: startNote resets the pressure
    case Event::Kind::PitchBend:
    case Event::Kind::ModWheel:
    case Event::Kind::Aftertouch:
    case Event::Kind::Sustain:
    case Event::Kind::AllNotesOff:
    case Event::Kind::KillAll:
        return 4;
    }
    return 4;
}

void EngineCore::recordNote(int note, int velocity)
{
    if (!recording)
        return;
    RecordEvent r;
    r.note = note;
    r.velocity = velocity;
    if (seqWasActive)
    {
        const auto p = seq.lastPosition();
        r.step = p.index;
        r.slot = p.slot;
    }
    recordRing.push(r);
}

KeyLockSettings EngineCore::keySettings() const
{
    const std::uint64_t v = keyPacked.load(std::memory_order_relaxed);
    KeyLockSettings s;
    s.mask = static_cast<std::uint16_t>(v & 0x0FFFU);
    s.root = static_cast<int>((v >> 12) & 0xFU) % 12;
    s.tieUp = ((v >> 16) & 1U) != 0;
    s.mode = static_cast<KeyMode>((v >> 17) & 3U);
    if (v == 0)
        s = KeyLockSettings{};
    return s;
}

void EngineCore::updateKeyLock(const float* values)
{
    KeyLockSettings s;
    s.mode = static_cast<KeyMode>(std::clamp(static_cast<int>(values[idx.keyMode]), 0, 3));
    s.root = std::clamp(static_cast<int>(values[idx.keyRoot]), 0, 11);
    const int scale = std::clamp(static_cast<int>(values[idx.keyScale]), 0, kNumKeyScales - 1);
    s.mask = scale == kKeyScaleCustom
                 ? static_cast<std::uint16_t>(std::clamp(static_cast<int>(values[idx.keyMask]), 0, 4095))
             : scale == kKeyScaleTuning ? static_cast<std::uint16_t>(synth.tuning().scaleMask())
                                        : keyScaleMask(scale);
    s.tieUp = values[idx.keyTie] > 0.5f;
    const int sync = static_cast<int>(values[idx.keySync]);
    KeyBus* bus = keyBus.load(std::memory_order_acquire);
    bool follow = false;
    if (bus != nullptr && sync == 1)
    {
        const KeyBusState st = bus->read();
        if (st.generation != 0)
        {
            s.root = st.root;
            s.mask = st.mask;
            s.tieUp = st.tieUp;
            follow = true;
        }
        haveLead = false;
    }
    else if (bus != nullptr && sync == 2)
    {
        if (!haveLead || s.root != leadSent.root || s.mask != leadSent.mask || s.tieUp != leadSent.tieUp)
        {
            bus->publish(s.root, s.mask, s.tieUp, writerId);
            leadSent = s;
            haveLead = true;
        }
    }
    else
        haveLead = false;
    quantiser.configure(s);
    arpQuantiser.configure(s);
    keyPacked.store(static_cast<std::uint64_t>(s.mask) | (static_cast<std::uint64_t>(s.root) << 12) |
                        (s.tieUp ? 1ULL << 16 : 0ULL) | (static_cast<std::uint64_t>(s.mode) << 17) | (1ULL << 20),
                    std::memory_order_relaxed);
    following.store(follow, std::memory_order_relaxed);
}

void EngineCore::markSounding(int note, bool on)
{
    if (note < 0 || note > 127)
        return;
    const auto w = static_cast<size_t>(note >> 6);
    const std::uint64_t bit = std::uint64_t{1} << (note & 63);
    if (on)
    {
        soundingNow[w] |= bit;
        soundingTouched[w] |= bit;
    }
    else
        soundingNow[w] &= ~bit;
}

int EngineCore::startKey(NoteQuantiser& q, int note, float velocity, int channel)
{
    const auto r = q.noteOn(note);
    if (r.stop >= 0)
        synth.noteOff(r.stop);
    if (r.out >= 0)
        synth.noteOn(r.out, velocity, channel);
    return r.out;
}

int EngineCore::stopKey(NoteQuantiser& q, int note)
{
    const auto r = q.noteOff(note);
    if (r.release)
    {
        synth.noteOff(r.out);
        return r.out;
    }
    if (!r.known && q.config().mode == KeyMode::Off)
    {
        synth.noteOff(note); // never started here: behave as an engine without key lock
        return note;
    }
    return -1;
}

void EngineCore::apply(const Event& e, bool& dirty)
{
    switch (e.kind)
    {
    case Event::Kind::NoteOn:
        detector.noteOn(e.a, e.f);
        startKey(quantiser, e.a, e.f, e.channel);
        recordNote(e.a, std::clamp(static_cast<int>(e.f * 127.0f + 0.5f), 1, 127));
        break;
    case Event::Kind::SeqNoteOn:
        markSounding(startKey(quantiser, e.a, e.f, -1), true);
        break;
    case Event::Kind::NoteOff:
        detector.noteOff(e.a);
        stopKey(quantiser, e.a);
        break;
    case Event::Kind::SeqNoteOff:
        markSounding(stopKey(quantiser, e.a), false);
        break;
    case Event::Kind::PitchBend:
        if (mpeActive && e.channel > 0)
            synth.setChannelBend(e.channel, e.f);
        else
            synth.setPitchBendNormalised(e.f);
        break;
    case Event::Kind::ModWheel:
        synth.setModWheel(e.f);
        break;
    case Event::Kind::Aftertouch:
        if (mpeActive && e.channel > 0)
            synth.setChannelPressure(e.channel, e.f);
        else
            synth.setAftertouch(e.f);
        break;
    case Event::Kind::PolyPressure:
        synth.setNotePressure(e.a, e.f);
        break;
    case Event::Kind::Slide:
        synth.setChannelSlide(e.channel, e.f);
        break;
    case Event::Kind::Sustain:
        synth.setSustainPedal(e.f > 0.5f);
        break;
    case Event::Kind::AllNotesOff:
        quantiser.reset();
        synth.allNotesOff();
        soundingNow = {};
        break;
    case Event::Kind::KillAll:
        quantiser.reset();
        synth.killAllVoices();
        soundingNow = {};
        break;
    case Event::Kind::LockSet:
        if (e.a >= 0 && static_cast<size_t>(e.a) < locked.size())
        {
            locked[static_cast<size_t>(e.a)] = e.f;
            lockActive[static_cast<size_t>(e.a)] = 1;
            refreshExclusions(static_cast<size_t>(e.a));
            dirty = true;
        }
        break;
    case Event::Kind::LockClear:
        if (e.a >= 0 && static_cast<size_t>(e.a) < locked.size())
        {
            lockActive[static_cast<size_t>(e.a)] = 0;
            refreshExclusions(static_cast<size_t>(e.a));
            dirty = true;
        }
        break;
    case Event::Kind::Position:
        seqStep.store(e.a, std::memory_order_relaxed);
        break;
    case Event::Kind::Controller:
        synth.setController(e.a, e.f);
        break;
    }
}

void EngineCore::process(float* left, float* right, int numSamples, const float* values, const MidiInput* midi, int numMidi,
                         const SeqHostTime& host, const float* auxLeft, const float* auxRight)
{
    const ScopedFlushDenormals noDenormals; // hosts that call the core directly get the same protection
    blocks.fetch_add(1, std::memory_order_relaxed);
    soundingTouched = soundingNow;
    const size_t n = working.size();
    std::copy(values, values + n, working.begin());
    for (size_t i = 0; i < n; ++i)
        if (lockActive[i] != 0)
            working[i] = locked[i];
    synth.setParameterValues(working.data(), static_cast<int>(n));
    if (host.hasBpm)
        synth.setTempo(host.bpm);
    synth.setHostTransport(host.hasPpq, host.playing, host.ppq);
    outputMeter.setHoldSeconds(values[idx.hold]);
    recording = values[idx.rec] > 0.5f;
    mpeActive = values[idx.mpeOn] > 0.5f;
    updateKeyLock(values);

    int count = 0;
    const bool arpOn = values[idx.arpOn] > 0.5f;
    int arpCount = 0;
    const auto add = [&](int pos, Event::Kind kind, int a, float f, int channel = -1)
    {
        // With the arpeggiator on, keyboard notes feed it instead of the voices (they are still step-recorded).
        if (arpOn && (kind == Event::Kind::NoteOn || kind == Event::Kind::NoteOff))
        {
            const int apos = std::clamp(pos, 0, std::max(numSamples - 1, 0));
            const auto pushArp = [&](bool on, int note, float vel)
            {
                if (arpCount < static_cast<int>(arpIn.size()))
                    arpIn[static_cast<size_t>(arpCount++)] = {apos, on, note, vel};
            };
            if (kind == Event::Kind::NoteOn)
            {
                recordNote(a, std::clamp(static_cast<int>(f * 127.0f + 0.5f), 1, 127));
                const auto r = arpQuantiser.noteOn(a);
                if (r.stop >= 0)
                    pushArp(false, r.stop, 0.0f);
                if (r.out >= 0)
                    pushArp(true, r.out, f);
            }
            else
            {
                const auto r = arpQuantiser.noteOff(a);
                if (r.release)
                    pushArp(false, r.out, f);
                else if (!r.known && arpQuantiser.config().mode == KeyMode::Off)
                    pushArp(false, a, f);
            }
            return;
        }
        if (arpOn && arpWasOn && (kind == Event::Kind::AllNotesOff || kind == Event::Kind::KillAll))
        {
            // Panic reaches the arpeggiator's held keys too, or it would go on playing the chord. (Not on the block that
            // switches the arpeggiator on: it starts empty, and the notes of that block are already queued for it.)
            if (arpCount < static_cast<int>(arpIn.size()))
            {
                ArpInput panic;
                panic.position = std::clamp(pos, 0, std::max(numSamples - 1, 0));
                panic.panic = true;
                arpIn[static_cast<size_t>(arpCount++)] = panic;
            }
            arpQuantiser.reset();
        }
        if (count < static_cast<int>(events.size()))
        {
            Event& e = events[static_cast<size_t>(count++)];
            e.position = std::clamp(pos, 0, std::max(numSamples - 1, 0));
            e.order = priority(kind);
            e.kind = kind;
            e.a = a;
            e.f = f;
            e.channel = channel;
        }
    };

    // External (OSC keyboard, UI) events apply at the start of the block.
    ExternalEvent x;
    while (externalRing.pop(x))
    {
        switch (x.type)
        {
        case ExternalEvent::Type::NoteOn:
            add(0, Event::Kind::NoteOn, x.note, x.value, x.channel);
            break;
        case ExternalEvent::Type::NoteOff:
            add(0, Event::Kind::NoteOff, x.note, 0.0f, x.channel);
            break;
        case ExternalEvent::Type::PitchBend:
            add(0, Event::Kind::PitchBend, 0, x.value, x.channel);
            break;
        case ExternalEvent::Type::ModWheel:
            add(0, Event::Kind::ModWheel, 0, x.value);
            break;
        case ExternalEvent::Type::Sustain:
            add(0, Event::Kind::Sustain, 0, x.value);
            break;
        case ExternalEvent::Type::AllNotesOff:
            add(0, Event::Kind::AllNotesOff, 0, 0.0f);
            break;
        case ExternalEvent::Type::KillAll:
            add(0, Event::Kind::KillAll, 0, 0.0f);
            break;
        case ExternalEvent::Type::Aftertouch:
            add(0, Event::Kind::Aftertouch, 0, x.value, x.channel);
            break;
        case ExternalEvent::Type::Controller:
            if (mpeActive && x.channel > 0 && x.note == 74)
                add(0, Event::Kind::Slide, 0, x.value, x.channel);
            else
                add(0, Event::Kind::Controller, x.note, x.value);
            break;
        case ExternalEvent::Type::PolyPressure:
            add(0, Event::Kind::PolyPressure, x.note, x.value);
            break;
        }
    }

    for (int i = 0; i < numMidi; ++i)
    {
        const auto& m = midi[i];
        const std::uint8_t type = m.status & 0xf0U;
        const int ch = m.status & 0x0f;
        if (type == 0x90U && m.data2 > 0)
            add(m.position, Event::Kind::NoteOn, m.data1, static_cast<float>(m.data2) / 127.0f, ch);
        else if (type == 0x80U || type == 0x90U)
            add(m.position, Event::Kind::NoteOff, m.data1, 0.0f, ch);
        else if (type == 0xe0U)
            add(m.position, Event::Kind::PitchBend, 0, (static_cast<float>(m.data1 | (m.data2 << 7U)) - 8192.0f) / 8192.0f,
                ch);
        else if (type == 0xd0U)
            add(m.position, Event::Kind::Aftertouch, 0, static_cast<float>(m.data1) / 127.0f, ch);
        else if (type == 0xa0U)
            add(m.position, Event::Kind::PolyPressure, m.data1, static_cast<float>(m.data2) / 127.0f);
        else if (type == 0xb0U)
        {
            const bool member = mpeActive && ch > 0;
            if (member && m.data1 == 74)
            {
                add(m.position, Event::Kind::Slide, 0, static_cast<float>(m.data2) / 127.0f, ch);
                continue;
            }
            if (member && (m.data1 == 1 || m.data1 == 64))
                continue; // wheel and pedal belong to the master channel in MPE
            if (m.data1 == 1)
                add(m.position, Event::Kind::ModWheel, 0, static_cast<float>(m.data2) / 127.0f);
            else if (m.data1 == 64)
                add(m.position, Event::Kind::Sustain, 0, m.data2 >= 64 ? 1.0f : 0.0f);
            else if (m.data1 == 123 || m.data1 == 120)
                add(m.position, Event::Kind::AllNotesOff, 0, 0.0f);
            // Every controller (including the ones above) is also a source for assignable modulation routes.
            add(m.position, Event::Kind::Controller, m.data1 & 0x7f, static_cast<float>(m.data2 & 0x7f) / 127.0f);
        }
    }

    // Sequencer.
    SeqSettings ss;
    ss.run = values[idx.run] > 0.5f;
    ss.hostSync = values[idx.sync] > 0.5f;
    ss.tempo = values[idx.tempo];
    ss.rate = static_cast<int>(values[idx.rate]);
    ss.direction = static_cast<int>(values[idx.dir]);
    ss.swingPercent = values[idx.swing];
    ss.transpose = static_cast<int>(values[idx.transpose]);
    ss.pattern = static_cast<int>(values[idx.pattern]);
    ss.chain = static_cast<int>(values[idx.chain]);
    const SeqBank* bank = seqStore.acquire();
    const int seqCount = seq.process(*bank, ss, host, numSamples, seqEvents.data(), static_cast<int>(seqEvents.size()));
    seqStore.release();
    seqWasActive = seq.isActive();
    seqRunning.store(seqWasActive, std::memory_order_relaxed);
    for (int i = 0; i < seqCount; ++i)
    {
        const auto& s = seqEvents[static_cast<size_t>(i)];
        switch (s.type)
        {
        case SeqEvent::Type::NoteOn:
            add(s.offset, Event::Kind::SeqNoteOn, s.a, s.value);
            break;
        case SeqEvent::Type::NoteOff:
            add(s.offset, Event::Kind::SeqNoteOff, s.a, 0.0f);
            break;
        case SeqEvent::Type::LockSet:
            add(s.offset, Event::Kind::LockSet, s.a, s.value);
            break;
        case SeqEvent::Type::LockClear:
            add(s.offset, Event::Kind::LockClear, s.a, 0.0f);
            break;
        case SeqEvent::Type::Position:
            add(s.offset, Event::Kind::Position, s.a, 0.0f);
            break;
        }
    }

    // Arpeggiator.
    if (arpOn != arpWasOn)
    {
        if (arpOn)
            add(0, Event::Kind::AllNotesOff, 0, 0.0f); // notes held before the switch would hang
        else
        {
            const int n2 = arp.flush(arpOut.data(), static_cast<int>(arpOut.size()));
            for (int i = 0; i < n2; ++i)
                add(0, Event::Kind::SeqNoteOff, arpOut[static_cast<size_t>(i)].note, 0.0f);
            arp.reset();
        }
        arpQuantiser.reset();
        arpWasOn = arpOn;
    }
    if (arpOn)
    {
        // Insertion sort (stable, allocation free): external events sit at 0, MIDI is already in order.
        for (int i = 1; i < arpCount; ++i)
        {
            const ArpInput key = arpIn[static_cast<size_t>(i)];
            int j = i - 1;
            while (j >= 0 && arpIn[static_cast<size_t>(j)].position > key.position)
            {
                arpIn[static_cast<size_t>(j + 1)] = arpIn[static_cast<size_t>(j)];
                --j;
            }
            arpIn[static_cast<size_t>(j + 1)] = key;
        }
        ArpSettings as;
        as.on = true;
        as.mode = static_cast<int>(values[idx.arpMode]);
        as.rate = static_cast<int>(values[idx.arpRate]);
        as.octaves = static_cast<int>(values[idx.arpOctaves]);
        as.gate = values[idx.arpGate];
        as.latch = values[idx.arpLatch] > 0.5f;
        as.hostSync = values[idx.arpSync] > 0.5f;
        as.tempo = values[idx.arpTempo];
        as.freeRateHz = values[idx.arpFree];
        as.swingPercent = values[idx.arpSwing];
        as.ratchet = static_cast<int>(values[idx.arpRatchet]);
        as.transpose = static_cast<int>(values[idx.arpTranspose]);
        as.velocityMode = static_cast<int>(values[idx.arpVelMode]);
        as.fixedVelocity = static_cast<int>(values[idx.arpVel]);
        as.accent = values[idx.arpAccent];
        as.useGrid = values[idx.arpGrid] > 0.5f;
        const ArpPattern* grid = arpStore.acquire();
        const ArpPattern* use = grid;
        ArpPattern patched; // fixed size, on the stack
        if (grid != nullptr && as.useGrid)
        {
            // The grid length and direction parameters drive the stored steps.
            patched.steps = grid->steps;
            patched.length = static_cast<int>(values[idx.arpGridLength]);
            patched.direction = static_cast<int>(values[idx.arpGridDir]);
            use = &patched;
        }
        const int n2 =
            arp.process(arpIn.data(), arpCount, as, use, host, numSamples, arpOut.data(), static_cast<int>(arpOut.size()));
        arpStore.release();
        for (int i = 0; i < n2; ++i)
        {
            const auto& o = arpOut[static_cast<size_t>(i)];
            add(o.offset, o.on ? Event::Kind::SeqNoteOn : Event::Kind::SeqNoteOff, o.note, o.velocity);
        }
    }

    // Stable insertion sort by position, then priority (the buffer is small and mostly sorted).
    for (int i = 1; i < count; ++i)
    {
        const Event key = events[static_cast<size_t>(i)];
        int j = i - 1;
        while (j >= 0 && (events[static_cast<size_t>(j)].position > key.position ||
                          (events[static_cast<size_t>(j)].position == key.position &&
                           events[static_cast<size_t>(j)].order > key.order)))
        {
            events[static_cast<size_t>(j + 1)] = events[static_cast<size_t>(j)];
            --j;
        }
        events[static_cast<size_t>(j + 1)] = key;
    }

    int position = 0;
    int next = 0;
    while (position < numSamples || next < count)
    {
        const int until = next < count ? events[static_cast<size_t>(next)].position : numSamples;
        if (until > position)
        {
            synth.setAuxInput(auxLeft != nullptr ? auxLeft + position : nullptr,
                              auxRight != nullptr ? auxRight + position : nullptr);
            synth.render(left + position, right + position, until - position);
            position = until;
        }
        if (next >= count)
            break;
        bool dirty = false;
        while (next < count && events[static_cast<size_t>(next)].position <= position)
            apply(events[static_cast<size_t>(next++)], dirty);
        if (dirty)
        {
            std::copy(values, values + n, working.begin());
            for (size_t i = 0; i < n; ++i)
                if (lockActive[i] != 0)
                    working[i] = locked[i];
            synth.setParameterValues(working.data(), static_cast<int>(n));
        }
    }
    sounding[0].store(soundingTouched[0], std::memory_order_relaxed);
    sounding[1].store(soundingTouched[1], std::memory_order_relaxed);
    detector.tick(static_cast<float>(numSamples / rate));
    outputMeter.process(left, right, numSamples);
    masterAnalyzer.process(left, right, numSamples);
    viewHub.publish(numSamples);
}

} // namespace crucible
