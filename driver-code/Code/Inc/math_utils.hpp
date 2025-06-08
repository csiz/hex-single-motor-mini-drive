#pragma once

#include <cmath> 
#include <cstdint>

// Square a number.
inline constexpr int square(int x){
    return x * x;
}

inline constexpr int abs(int x){
    return x < 0 ? -x : x;
}

// Get the smaller between two numbers.
inline constexpr int min(int a, int b){
    return a < b ? a : b;
}

// Get the larger between two numbers.
inline constexpr int max(int a, int b){
    return a > b ? a : b;
}

// Clip a value between two limits; params are (low high value).
inline constexpr int clip_to(int low, int high, int value){
    return min(high, max(low, value));
}

inline constexpr int sign(int x){
    return (x > 0) - (x < 0);
}