#pragma once

#include <cstdint> // For int16_t, int32_t
#include <array>
#include <utility> // For std::pair

#include "constants.hpp"
#include "math_utils.hpp"


// Square root of integer (straight from https://en.wikipedia.org/wiki/Integer_square_root).
static inline int int_sqrt(int s)
{
	// Zero yields zero
    // One yields one
	if (s <= 1) 
		return s;

    // Initial estimate (must be too high)
	int x0 = s / 2;

	// Update
	int x1 = (x0 + s / x0) / 2;

	while (x1 < x0)	// Bound check
	{
		x0 = x1;
		x1 = (x0 + s / x0) / 2;
	}		
	return x0;
}

// Minimum magnitude for funky atan2 to be considered valid
const int funky_atan2_constant = 4;

static inline int funky_atan2(int y, int x) {
    // In mod angle_base arithmetic this is equivalent to 0, but allows us to distinguish the 0 case.    
    if (x == 0 and y == 0) return angle_base;

    int result = 0;
    if (x < 0) {
        // Rotate 180 degrees and compensate in the result.
        result += half_circle;
        x = -x;
        y = -y;
    }

    if (y < 0) {
        // Rotate 270 degrees and compensate in the result.
        result += three_quarters_circle;
        int temp = x;
        x = -y;
        y = temp;
    }
    
    // Now x >= 0 and y >= 0.

    if (x >= y) {
        result += (funky_atan2_constant * y * half_circle_div_pi) / (funky_atan2_constant * x + y);
    } else {
        result += quarter_circle - (funky_atan2_constant * x * half_circle_div_pi) / (funky_atan2_constant * y + x);
    }

    // Return the result sign normalized to [-half_circle to +half_circle].
    return (result + half_circle) % angle_base - half_circle;
}


void unit_test_funky_atan(char * buffer, size_t max_size);
void unit_test_funky_atan_part_2(char * buffer, size_t max_size);
void unit_test_funky_atan_part_3(char * buffer, size_t max_size);

static inline int angle_or_mirror(const int angle){
    return (
        angle >= 0 ? (
            angle <= quarter_circle ?
                angle :
                angle - half_circle
        ) : (
            angle >= -quarter_circle ?
                angle :
                angle + half_circle
        )
    );
}
