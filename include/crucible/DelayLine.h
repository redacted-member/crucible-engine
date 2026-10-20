// SPDX-License-Identifier: MIT
#pragma once

#include <vector>

namespace crucible
{

// Fractional delay line with cubic interpolation.
class DelayLine
{
public:
    void allocate(int minimumSamples);
    void clear();
    // Clears at most `budget` more samples of the buffer (and takes them off `budget`); true once the whole buffer is
    // clear. The audio thread uses it to spread a long clear over many blocks.
    bool clearStep(int& budget);
    void write(float x)
    {
        buffer[static_cast<size_t>(pos)] = x;
        pos = (pos + 1) & mask;
    }
    // delay >= 1 sample (counted from the most recently written sample)
    float read(float delay) const;
    int capacity() const { return mask + 1; }

private:
    std::vector<float> buffer{std::vector<float>(2, 0.0f)};
    int pos = 0;
    int mask = 1;
    int clearCursor = 0;
};

} // namespace crucible
