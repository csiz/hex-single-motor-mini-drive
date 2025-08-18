#pragma once

// Square a number.
static inline constexpr int square(int x){
    return x * x;
}

// Get the smaller between two numbers.
static inline constexpr int min(int a, int b){
    return a < b ? a : b;
}


// Get the larger between two numbers.
static inline constexpr int max(int a, int b){
    return a > b ? a : b;
}

static inline constexpr int max(int a, int b, int c){
    return max(max(a, b), c);
}

// Clip a value between two limits; params are (low, high, value).
static inline constexpr int clip_to(int low, int high, int value){
    return min(high, max(low, value));
}

// Clip a value to a signed 16-bit range.
static inline constexpr int16_t clip_to_short(int low, int high, int value){
    return static_cast<int16_t>(clip_to(low, high, value));
}

// Sign of x or 0 if x is 0.
static inline constexpr int sign(int x){
    return (x > 0) - (x < 0);
}

// Divide x by y and round to the nearest integer.
static inline constexpr int round_div(const int x, const int y) {
    return (x + y / 2) / y;
}

// Divide x by y and round to the nearest integer away from zero.
static inline constexpr int signed_ceil_div(const int x, const int y) {
    return sign(x) + (x / y);
}

// Get the absolute value of a number; somehow the std implementation is slower.
static inline constexpr int faster_abs(int value) {
    return value < 0 ? -value : value;
}