#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>


// Most useful datatype for this system, the 2 phase coordinates.
using TwoPhase = std::tuple<float, float>;

// Second most useful is the three phase coords.
using ThreePhase = std::tuple<float, float, float>;


// Dot product between three phase float vectors.
static inline float dot(ThreePhase const& a, ThreePhase const& b) {
    return (
        std::get<0>(a) * std::get<0>(b) +
        std::get<1>(a) * std::get<1>(b) +
        std::get<2>(a) * std::get<2>(b)
    );
}

constexpr float one_third = 1.0 / 3.0;

constexpr float two_thirds = 2.0 / 3.0;

// DQ0 transform coefficients to the fixed angle 0.

// D transform row.
constexpr ThreePhase d_transform = {
    1.0f * two_thirds,
    -0.5f * two_thirds,
    -0.5f * two_thirds
};

// Q transform row.
constexpr ThreePhase q_transform = {
    0.0f * two_thirds,
    0.86602540378f * two_thirds, // sin(120 degrees)
    -0.86602540378f * two_thirds // sin(-120 degrees)
};

// Compute the dq0 transform of a three phase vector using the reference angle 0.
static inline TwoPhase dq0_transform(ThreePhase const& abc) {
    return {
        dot(abc, d_transform),
        dot(abc, q_transform)
    };
}

// Compute the inverse to the dq0 transform.
static inline ThreePhase abc_transform(TwoPhase const& dq, float const& zero_offset) {
    return {
        std::get<0>(dq) + zero_offset,
        -0.5f * std::get<0>(dq) + 0.86602540378f * std::get<1>(dq) + zero_offset,
        -0.5f * std::get<0>(dq) - 0.86602540378f * std::get<1>(dq) + zero_offset
    };
}