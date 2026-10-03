#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

#include "hex_mini_drive_interface.hpp"

// Most useful datatype for this system, the 2 phase coordinates.
using TwoPhase = std::tuple<float, float>;

// Second most useful is the three phase coords.
using ThreePhase = std::tuple<float, float, float>;

// Function signature for unit tests; the unit test prints text to the buffer.
using UnitTestFunction = void (*)(char * buffer, size_t max_size);

// ADC Readings
// ------------

struct ADCReadings {
    int16_t u_readout;
    int16_t v_readout;
    int16_t w_readout;
    int16_t ref_readout;
    int16_t temp_readout;
    int16_t vcc_readout;
};


// Driver State
// ------------

// Motor duty cycle (compare register values and enable settings).
struct MotorOutputs {
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
    // TODO: add the 4th phase
    // uint16_t z_duty; // PWM duty cycle for Z phase (4 wire servos).
};

// Enable all motor outputs (U, V, W).
const uint8_t enable_flags_all = 0b1111;
// Enable U phase outputs.
const uint8_t enable_flags_u = 0b0001;
// Enable V phase outputs.
const uint8_t enable_flags_v = 0b0010;
// Enable W phase outputs.
const uint8_t enable_flags_w = 0b0100;
// Enable Z phase outputs.
const uint8_t enable_flags_z = 0b1000;
// Disable all motor outputs.
const uint8_t enable_flags_none = 0b0000;
// Enable recording to the readout history. This flag allows us to synchornize recordings with new commands.
const uint8_t enable_recording = 0b0001'0000;

// Zeroed motor outputs, used to short circuit break the motor outputs.
const MotorOutputs breaking_motor_outputs = {
    .u_duty = 0,
    .v_duty = 0,
    .w_duty = 0,
};

// The complete driver state, these values control the motor behaviour.
struct DriverState {
    // Control flags for the driver (which motor outputs are enabled).
    uint8_t motor_flags = enable_flags_all;

    // Dedicated bool value for closed loop control mode.
    uint8_t enable_closed_loop;

    // Duration for the command in pwm cycles.
    uint16_t duration;
    
    // PWM value actively used to drive the motor.
    float pwm_value;
    
    // Incremental change for the PWM drive.
    float pwm_change;

    // Angle at which the motor is currently driven.
    int32_t angle;
    
    // The angular speed of the driving wave.
    float angular_speed;
};

// Driver state for safely breaking the motor.
const DriverState breaking_driver_state = {
    .motor_flags = enable_flags_all,
};

// Driver state for electrically disconnecting the motor.
const DriverState freewheel_driver_state = {
    .motor_flags = enable_flags_none,
};

// Size of the DriverState structure (we want to keep it small for performance).
const size_t driver_state_size = sizeof(DriverState);

static_assert(driver_state_size <= 64, "DriverState size exceeds 64 bytes, try to make it smaller!");



// Three phase arithmetic operators
// --------------------------------


// Element-wise difference.
static inline ThreePhase operator - (ThreePhase const& a, ThreePhase const& b) {
    return ThreePhase {
        std::get<0>(a) - std::get<0>(b),
        std::get<1>(a) - std::get<1>(b),
        std::get<2>(a) - std::get<2>(b)
    };
}

// Element-wise addition.
static inline ThreePhase operator + (ThreePhase const& a, ThreePhase const& b) {
    return ThreePhase {
        std::get<0>(a) + std::get<0>(b),
        std::get<1>(a) + std::get<1>(b),
        std::get<2>(a) + std::get<2>(b)
    };
}

// Scalar multiplication.
static inline ThreePhase operator * (ThreePhase const& a, float b) {
    return ThreePhase {
        std::get<0>(a) * b,
        std::get<1>(a) * b,
        std::get<2>(a) * b
    };
}

// Element-wise multiplication.
static inline ThreePhase operator * (ThreePhase const& a, ThreePhase const& b) {
    return ThreePhase {
        std::get<0>(a) * std::get<0>(b),
        std::get<1>(a) * std::get<1>(b),
        std::get<2>(a) * std::get<2>(b)
    };
}

// Scalar division.
static inline ThreePhase operator / (ThreePhase const& a, float b) {
    return ThreePhase {
        std::get<0>(a) / b,
        std::get<1>(a) / b,
        std::get<2>(a) / b
    };
}

// Element-wise division.
static inline ThreePhase operator / (ThreePhase const& a, ThreePhase const& b) {
    return ThreePhase {
        std::get<0>(a) / std::get<0>(b),
        std::get<1>(a) / std::get<1>(b),
        std::get<2>(a) / std::get<2>(b)
    };
}

// Dot product between three phase float vectors.
static inline float dot(ThreePhase const& a, ThreePhase const& b) {
    return (
        std::get<0>(a) * std::get<0>(b) +
        std::get<1>(a) * std::get<1>(b) +
        std::get<2>(a) * std::get<2>(b)
    );
}


// Three phase helper functions
// ----------------------------

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