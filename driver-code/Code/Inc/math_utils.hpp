#pragma once

#include <cmath> 
#include <cstdint>

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

// Clip a value between two limits; params are (low high value).
static inline constexpr int clip_to(int low, int high, int value){
    return min(high, max(low, value));
}

static inline constexpr int sign(int x){
    return (x > 0) - (x < 0);
}


static inline constexpr int round_div(const int x, const int y) {
    // Round the division to the nearest integer.
    return (x + y / 2) / y;
}

static inline constexpr int signed_round_div(const int x, const int y) {
    // Round the division to the nearest integer, preserving the sign.
    return (x >= 0 ? (x + y / 2) : (x - y / 2)) / y;
}