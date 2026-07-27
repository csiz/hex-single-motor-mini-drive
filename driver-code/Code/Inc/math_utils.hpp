#pragma once

// Square a number.
#include <cstdint>

static inline constexpr float square(float x){
    return x * x;
}

// Get the smaller between two numbers.
static inline constexpr int32_t min(int32_t a, int32_t b){
    return a < b ? a : b;
}

// Get the smaller between two numbers.
static inline constexpr float min(float a, float b){
    return a < b ? a : b;
}

// Get the larger between two numbers.
static inline constexpr int32_t max(int32_t a, int32_t b){
    return a > b ? a : b;
}

// Get the larger between two numbers.
static inline constexpr float max(float a, float b){
    return a > b ? a : b;
}

static inline constexpr float max(float a, float b, float c, float d){
    return max(max(a, b), max(c, d));
}

// Clip a value between two limits; params are (low, high, value).
static inline constexpr int32_t clip_to(int32_t low, int32_t high, int32_t value){
    return min(high, max(low, value));
}

// Clip a value between two limits; params are (low, high, value).
static inline constexpr float clip_to(float const& low, float const& high, float const& value){
    return min(high, max(low, value));
}

// Sign of x or 0 if x is 0.
static inline constexpr int32_t sign(int32_t const& x){
    return (x > 0) - (x < 0);
}

static inline constexpr int16_t sign(int16_t const& x){
    return (x > 0) - (x < 0);
}

static inline constexpr float sign(float const& x){
    return (x > 0.f) - (x < 0.f);
}

// Divide x by y and round to the nearest integer.
static inline constexpr int round_div(const int x, const int y) {
    return (x + y / 2) / y;
}


// Get the absolute value of a number; somehow the std implementation is slower.
static inline constexpr int faster_abs(int value) {
    return value < 0 ? -value : value;
}

// Get the absolute value of a number; somehow the std implementation is slower.
static inline constexpr int faster_abs(int16_t value) {
    return value < 0 ? -value : value;
}

// Get the absolute value of a number; somehow the std implementation is slower.
static inline constexpr float faster_abs(float value) {
    return value < 0.f ? -value : value;
}