// SPDX-License-Identifier: MIT
#include <crucible/graph/Program.h>

#include <crucible/module/ModuleRack.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace crucible::graph
{

Program::Program(std::shared_ptr<const CompiledProduct> product, const ModuleRack& rack, const ModuleContext& context,
                 std::uint32_t seed, ClassicVoiceHost* classicHost)
    : compiled(std::move(product))
{
    if (!compiled->buildFxPlan(rack, plan))
    {
        failure = "the FX graph does not fit this engine's effects";
        return;
    }
    build(context, seed);
    if (classic != nullptr)
        classic->bind(classicHost);
}

Program::~Program() = default;

void Program::build(const ModuleContext& context, std::uint32_t seed)
{
    baseSeed = seed;
    rate = context.sampleRate;
    chokeSamples = std::max(1, static_cast<int>(0.005 * rate));
    zeros.assign(kChunkSize, 0.0f);

    NodePrepare prep;
    prep.sampleRate = context.sampleRate;
    prep.maxBlock = kChunkSize;
    prep.quality = context.quality;
    prep.oversampling = context.oversampling;
    prep.batchPairs = context.batchAnalogFilters;

    if (compiled->voiceKind() == VoiceKind::Classic)
    {
        NodeCreate c;
        c.model = &compiled->registry->parameters();
        classicOwner = NodeRegistry::builtIn().find("classic.voice")->create(c);
        classic = static_cast<ClassicVoiceNode*>(classicOwner.get());
        return;
    }

    layerRt.resize(compiled->layers.size());
    for (size_t li = 0; li < compiled->layers.size(); ++li)
    {
        LayerRt& layer = layerRt[li];
        const LayerPlan& lp = compiled->layers[li];
        layer.plan = &lp;
        layer.voices.resize(static_cast<size_t>(lp.voices));
        for (size_t vi = 0; vi < layer.voices.size(); ++vi)
        {
            auto& voice = layer.voices[vi];
            voice.card = VoiceCard::make(seed + static_cast<std::uint32_t>(li) * 0x9e3779b9U, static_cast<int>(vi));
            voice.nodes.resize(lp.nodes.size());
            // Arena: stereo audio outputs and (for several sources) input sums, in chunks.
            size_t floats = 0;
            for (const auto& np : lp.nodes)
            {
                floats += static_cast<size_t>(np.numAudioOut) * 2 * kChunkSize;
                floats += np.rateIn.size() * kChunkSize;
                for (const auto& port : np.audioIn)
                    if (port.size() > 1 || (port.size() == 1 && port[0].amount != 1.0f))
                        floats += 2 * kChunkSize;
            }
            voice.arena.assign(floats, 0.0f);
            size_t at = 0;
            const auto grab = [&]()
            {
                float* p = voice.arena.data() + at;
                at += kChunkSize;
                return p;
            };
            for (size_t ni = 0; ni < lp.nodes.size(); ++ni)
            {
                const NodePlan& np = lp.nodes[ni];
                NodeRt& rt = voice.nodes[ni];
                rt.plan = &np;
                NodeCreate c;
                c.model = &compiled->registry->parameters();
                c.instanceId = np.id;
                c.slot = np.slot;
                c.oscillator = np.oscillator;
                c.paramIndex = np.params;
                rt.node = np.type->create(c);
                prep.index = static_cast<int>(ni);
                prep.card = &voice.card;
                prep.pairLead = vi % 2 == 0;
                prep.seed = seed + static_cast<std::uint32_t>(li + 1) * 0x9e3779b9U +
                            static_cast<std::uint32_t>(vi) * 0x85ebca6bU + static_cast<std::uint32_t>(ni) * 0xc2b2ae35U;
                rt.node->prepare(prep);
                rt.in.resize(np.audioIn.size());
                rt.out.resize(static_cast<size_t>(np.numAudioOut));
                rt.modIn.assign(np.modIn.size(), 0.0f);
                rt.modOut.assign(static_cast<size_t>(np.numModOut), 0.0f);
                for (auto& o : rt.out)
                {
                    o.left = grab();
                    o.right = grab();
                }
                rt.scratch.clear();
                rt.rate.clear();
                for (size_t p = 0; p < np.rateIn.size(); ++p)
                    rt.rate.push_back(grab());
                rt.rateView.assign(rt.rate.begin(), rt.rate.end());
                rt.io = NodeIO{};
                rt.io.in = rt.in.data();
                rt.io.numIn = static_cast<int>(rt.in.size());
                rt.io.out = rt.out.data();
                rt.io.numOut = static_cast<int>(rt.out.size());
                rt.io.modIn = rt.modIn.data();
                rt.io.numModIn = static_cast<int>(rt.modIn.size());
                rt.io.modOut = rt.modOut.data();
                rt.io.numModOut = static_cast<int>(rt.modOut.size());
                rt.io.paramIndex = np.params.data();
                for (size_t p = 0; p < np.modIn.size(); ++p)
                    if (!np.modIn[p].empty())
                        rt.io.modUsed |= 1U << p;
                rt.io.rateIn = rt.rateView.data();
                rt.io.numRateIn = static_cast<int>(rt.rateView.size());
                for (size_t p = 0; p < np.rateIn.size(); ++p)
                    if (!np.rateIn[p].empty())
                        rt.io.rateUsed |= 1U << p;
            }
            // Second pass: sum buffers (after the outputs, so the first pass offsets stay simple).
            for (size_t ni = 0; ni < lp.nodes.size(); ++ni)
            {
                const NodePlan& np = lp.nodes[ni];
                NodeRt& rt = voice.nodes[ni];
                for (size_t p = 0; p < np.audioIn.size(); ++p)
                {
                    const auto& port = np.audioIn[p];
                    if (port.size() > 1 || (port.size() == 1 && port[0].amount != 1.0f))
                    {
                        rt.in[p].left = grab();
                        rt.in[p].right = grab();
                    }
                    else if (port.empty())
                    {
                        rt.in[p].left = zeros.data();
                        rt.in[p].right = zeros.data();
                    }
                    else
                    {
                        const NodeRt& src = voice.nodes[static_cast<size_t>(port[0].node)];
                        rt.in[p] = src.out[static_cast<size_t>(port[0].port)];
                    }
                }
            }
        }
    }
}

int Program::activeVoices() const
{
    int n = 0;
    for (const auto& l : layerRt)
        for (const auto& v : l.voices)
            n += v.active ? 1 : 0;
    return n;
}

int Program::activeVoices(int layer) const
{
    int n = 0;
    if (layer >= 0 && layer < numLayers())
        for (const auto& v : layerRt[static_cast<size_t>(layer)].voices)
            n += v.active ? 1 : 0;
    return n;
}

void Program::killAll()
{
    for (auto& l : layerRt)
        for (auto& v : l.voices)
        {
            if (v.active)
                for (auto& n : v.nodes)
                    n.node->reset();
            v.active = false;
        }
}

void Program::triggerLayer(LayerRt& layer, int note, float velocity)
{
    // A free voice, else the quietest candidate: a choking one, then a released one, then the oldest.
    VoiceRt* pick = nullptr;
    for (auto& v : layer.voices)
        if (!v.active)
        {
            pick = &v;
            break;
        }
    if (pick == nullptr)
    {
        for (auto& v : layer.voices)
        {
            const auto rank = [](const VoiceRt& x) { return x.choking ? 0 : (x.released ? 1 : 2); };
            if (pick == nullptr || rank(v) < rank(*pick) || (rank(v) == rank(*pick) && v.age < pick->age))
                pick = &v;
        }
    }
    VoiceRt& v = *pick;
    v.active = true;
    v.released = false;
    v.choking = false;
    v.chokePos = 0;
    v.note = note;
    v.velocity = velocity;
    v.age = ++counter;
    NodeNote nn;
    nn.note = static_cast<float>(note);
    nn.velocity = velocity;
    nn.oneShot = layer.plan->oneShot;
    if (cardAmounts.any())
    {
        nn.card = &v.card;
        nn.cardAmounts = cardAmounts;
    }
    bool hold = false;
    for (size_t i = 0; i < v.nodes.size(); ++i)
    {
        nn.seed = baseSeed + static_cast<std::uint32_t>(v.age * 2654435761U) + static_cast<std::uint32_t>(i) * 0x9e3779b9U;
        v.nodes[i].node->reset();
        v.nodes[i].node->noteOn(nn);
        hold = hold || v.nodes[i].node->holdsVoice();
        std::fill(v.nodes[i].modOut.begin(), v.nodes[i].modOut.end(), 0.0f);
    }
    v.canHold = hold;
}

void Program::noteOn(int note, float velocity)
{
    if (compiled->voiceKind() != VoiceKind::Graph)
        return;
    const int vel = std::clamp(static_cast<int>(std::lround(velocity * 127.0f)), 1, 127);
    std::array<int, 16> rrSeen{};
    for (auto& layer : layerRt)
    {
        const LayerPlan& lp = *layer.plan;
        if (note < lp.noteLo || note > lp.noteHi || vel < lp.velLo || vel > lp.velHi)
            continue;
        if (lp.roundRobin != 0)
        {
            // Layers of one round-robin group that match alternate: this one plays when it is the turn's index.
            const int group = std::min(lp.roundRobin, 15);
            int matching = 0, mine = 0;
            for (const auto& other : layerRt)
            {
                const LayerPlan& o = *other.plan;
                if (o.roundRobin != lp.roundRobin || note < o.noteLo || note > o.noteHi || vel < o.velLo || vel > o.velHi)
                    continue;
                if (&other == &layer)
                    mine = matching;
                ++matching;
            }
            const auto turn =
                static_cast<int>(roundRobinCount[static_cast<size_t>(group)] % static_cast<std::uint64_t>(matching));
            if (turn != mine)
                continue;
            rrSeen[static_cast<size_t>(group)] = 1;
        }
        if (lp.chokeGroup != 0)
            for (auto& other : layerRt)
                if (other.plan->chokeGroup == lp.chokeGroup)
                    for (auto& v : other.voices)
                        if (v.active && !v.choking)
                        {
                            v.choking = true;
                            v.chokePos = 0;
                        }
        triggerLayer(layer, note, velocity);
    }
    for (size_t g = 0; g < rrSeen.size(); ++g)
        if (rrSeen[g] != 0)
            ++roundRobinCount[g];
}

void Program::noteOff(int note)
{
    for (auto& layer : layerRt)
    {
        if (layer.plan->oneShot)
            continue;
        for (auto& v : layer.voices)
            if (v.active && !v.released && v.note == note)
            {
                v.released = true;
                for (auto& n : v.nodes)
                    n.node->noteOff();
            }
    }
}

void Program::allNotesOff()
{
    for (auto& layer : layerRt)
        for (auto& v : layer.voices)
            if (v.active && !v.released)
            {
                v.released = true;
                for (auto& n : v.nodes)
                    n.node->noteOff();
            }
}

void Program::renderLayer(LayerRt& layer, float* left, float* right, int n, const float* values, const VoiceParams* shared,
                          float bend)
{
    const LayerPlan& lp = *layer.plan;
    std::array<VoiceRt*, 64> live{};
    int count = 0;
    for (auto& v : layer.voices)
        if (v.active && count < static_cast<int>(live.size()))
            live[static_cast<size_t>(count++)] = &v;
    if (count == 0)
        return;

    for (size_t ni = 0; ni < lp.nodes.size(); ++ni)
    {
        const NodePlan& np = lp.nodes[ni];
        for (int vi = 0; vi < count; ++vi)
        {
            VoiceRt& v = *live[static_cast<size_t>(vi)];
            NodeRt& rt = v.nodes[ni];
            for (size_t p = 0; p < np.audioIn.size(); ++p)
            {
                const auto& port = np.audioIn[p];
                if (port.size() > 1 || (port.size() == 1 && port[0].amount != 1.0f))
                {
                    float* l = rt.in[p].left;
                    float* r = rt.in[p].right;
                    std::fill(l, l + n, 0.0f);
                    std::fill(r, r + n, 0.0f);
                    for (const auto& s : port)
                    {
                        const AudioBuffer& src = v.nodes[static_cast<size_t>(s.node)].out[static_cast<size_t>(s.port)];
                        for (int i = 0; i < n; ++i)
                        {
                            l[i] += src.left[i] * s.amount;
                            r[i] += src.right[i] * s.amount;
                        }
                    }
                }
            }
            for (size_t p = 0; p < np.modIn.size(); ++p)
            {
                float sum = 0.0f;
                for (const auto& s : np.modIn[p])
                    sum += v.nodes[static_cast<size_t>(s.node)].modOut[static_cast<size_t>(s.port)] * s.amount;
                rt.modIn[p] = sum;
            }
            // Audio-rate inputs: the wired audio outputs mixed down to mono, scaled by their edge amounts.
            for (size_t p = 0; p < np.rateIn.size(); ++p)
            {
                float* buf = rt.rate[p];
                std::fill(buf, buf + n, 0.0f);
                for (const auto& s : np.rateIn[p])
                {
                    const AudioBuffer& src = v.nodes[static_cast<size_t>(s.node)].out[static_cast<size_t>(s.port)];
                    const float g = 0.5f * s.amount;
                    for (int i = 0; i < n; ++i)
                        buf[i] += (src.left[i] + (src.right != nullptr ? src.right[i] : src.left[i])) * g;
                }
            }
            rt.io.values = values;
            rt.io.note = static_cast<float>(v.note) + bend;
            rt.io.velocity = v.velocity;
            rt.io.shared = shared;
            rt.io.card = cardAmounts.any() ? &v.card : nullptr;
            rt.io.cardAmounts = cardAmounts;
        }
        if (layer.voices.front().nodes[ni].node->pairsVoices())
        {
            // Voices 2p and 2p + 1 of the pool share one four-lane object, so the pairing is by pool position and a
            // lane keeps its voice for the life of the program.
            for (size_t pi = 0; pi < layer.voices.size(); pi += 2)
            {
                VoiceRt& a = layer.voices[pi];
                VoiceRt* b = pi + 1 < layer.voices.size() ? &layer.voices[pi + 1] : nullptr;
                const bool soundA = a.active, soundB = b != nullptr && b->active;
                if (!soundA && !soundB)
                    continue;
                NodeRt& ra = a.nodes[ni];
                NodeRt* rb = b != nullptr ? &b->nodes[ni] : nullptr;
                if (ra.node->processPair(soundA ? &ra.io : nullptr, rb != nullptr ? rb->node.get() : nullptr,
                                         soundB ? &rb->io : nullptr, n))
                {
                    ++pairBlocks;
                    continue;
                }
                if (soundA)
                    ra.node->process(ra.io, n);
                if (soundB)
                    rb->node->process(rb->io, n);
            }
            continue;
        }
        // Four voices at a time where the node can; the rest one by one.
        int vi = 0;
        for (; vi + 4 <= count; vi += 4)
        {
            Node* peers[4];
            NodeIO* ios[4];
            for (int k = 0; k < 4; ++k)
            {
                NodeRt& rt = live[static_cast<size_t>(vi + k)]->nodes[ni];
                peers[k] = rt.node.get();
                ios[k] = &rt.io;
            }
            if (peers[0]->processQuad(peers, ios, n))
                continue;
            for (int k = 0; k < 4; ++k)
                peers[k]->process(*ios[k], n);
        }
        for (; vi < count; ++vi)
        {
            NodeRt& rt = live[static_cast<size_t>(vi)]->nodes[ni];
            rt.node->process(rt.io, n);
        }
    }

    for (int vi = 0; vi < count; ++vi)
    {
        VoiceRt& v = *live[static_cast<size_t>(vi)];
        const AudioBuffer& out = v.nodes[static_cast<size_t>(lp.sink)].out[0];
        if (v.choking)
        {
            for (int i = 0; i < n; ++i)
            {
                const float g = std::max(0.0f, 1.0f - static_cast<float>(v.chokePos + i) / static_cast<float>(chokeSamples));
                left[i] += out.left[i] * g;
                right[i] += out.right[i] * g;
            }
            v.chokePos += n;
            if (v.chokePos >= chokeSamples)
                v.active = false;
        }
        else
        {
            for (int i = 0; i < n; ++i)
            {
                left[i] += out.left[i];
                right[i] += out.right[i];
            }
        }
        if (v.active)
        {
            bool hold = false;
            for (const auto& nr : v.nodes)
                hold = hold || nr.node->holdsVoice();
            if (v.canHold ? !hold : v.released)
                v.active = false;
        }
        if (!v.active)
            for (auto& nr : v.nodes)
                nr.node->reset();
    }
}

void Program::render(float* left, float* right, int n, const float* values, const VoiceParams* shared, float bend)
{
    for (auto& layer : layerRt)
        renderLayer(layer, left, right, n, values, shared, bend);
}

// ---------------------------------------------------------------------------------------------------------------
ProgramHost::ProgramHost()
{
    held[0].store(nullptr);
    held[1].store(nullptr);
}

ProgramHost::~ProgramHost() = default;

Program* ProgramHost::publish(std::unique_ptr<Program> program)
{
    Program* raw = program.get();
    {
        std::lock_guard<std::mutex> lock(mutex);
        programs.push_back(std::move(program));
        latest = raw;
    }
    auto handle = std::make_unique<Handle>();
    handle->program = raw;
    shared.publish(std::move(handle));
    collect();
    return raw;
}

Program* ProgramHost::newest() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return latest;
}

int ProgramHost::liveCount() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return static_cast<int>(programs.size());
}

void ProgramHost::collect()
{
    shared.collectGarbage();
    std::lock_guard<std::mutex> lock(mutex);
    // The order matters: a handle the audio thread has pinned is read before the programs it has recorded, because it
    // records a program before it drops the pin.
    const Handle* pinned = shared.pinned();
    Program* pinnedProgram = pinned != nullptr ? pinned->program : nullptr;
    Program* a = held[0].load(std::memory_order_seq_cst);
    Program* b = held[1].load(std::memory_order_seq_cst);
    Program* c = candidate.load(std::memory_order_seq_cst);
    std::vector<std::unique_ptr<Program>> keep;
    for (auto& p : programs)
    {
        Program* raw = p.get();
        if (raw == latest || raw == pinnedProgram || raw == a || raw == b || raw == c)
            keep.push_back(std::move(p));
    }
    programs = std::move(keep);
}

Program* ProgramHost::poll() noexcept
{
    const Handle* h = shared.acquire();
    Program* p = h != nullptr ? h->program : nullptr;
    candidate.store(p, std::memory_order_seq_cst);
    shared.release();
    return p;
}

void ProgramHost::hold(int slot, Program* program) noexcept
{
    held[slot & 1].store(program, std::memory_order_seq_cst);
}

} // namespace crucible::graph
