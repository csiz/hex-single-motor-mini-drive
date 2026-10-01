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

// Driving modes for the motor control loop.
enum struct DriverMode : uint8_t {
    // All motor outputs are set to 0, the coils are all connected to ground slowing down the motor.
    GROUND_SHORT,
    // Set all motor outputs to maximum, the coils will still be connected together but through the high side switches.
    POSITIVE_SHORT,
    // Drive the motor with an open loop periodic excitation of the motor coils. The phases are
    // driven around a circle at the specified PWM and speed.
    DRIVE_FIXED_COORDINATES,
    // Drive the motor using FOC targeting a PWM value. This is the closed loop driving mode where
    // we aim to induce a current at the specified angle relative to the rotor position.
    DRIVE_RELATIVE_COORDINATES,
};

// Motor duty cycle (compare register values and enable settings).
struct MotorOutputs {
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
    // TODO: add the 4th phase
    // uint16_t z_duty; // PWM duty cycle for Z phase (4 wire servos).
};

// Enable all motor outputs (U, V, W).
const uint8_t enable_flags_all = 0b0111;
// Enable U phase outputs.
const uint8_t enable_flags_u = 0b0001;
// Enable V phase outputs.
const uint8_t enable_flags_v = 0b0010;
// Enable W phase outputs.
const uint8_t enable_flags_w = 0b0100;
// Disable all motor outputs.
const uint8_t enable_flags_none = 0b0000;

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

    // Motor driving mode (defaults to short circuit breaking).
    DriverMode mode;

    // Duration for the command in pwm cycles.
    uint16_t duration;

    // Angle at which the motor is currently driven.
    int32_t angle;
    
    // The angular speed of the driving wave.
    float angular_speed;

    // PWM value actively used to drive the motor.
    float pwm_value;

    // Incremental change for the PWM drive.
    float pwm_change;
};

// Driver state for safely breaking the motor.
const DriverState breaking_driver_state = {
    .motor_flags = enable_flags_all,
    .mode = DriverMode::GROUND_SHORT
};

// Driver state for electrically disconnecting the motor.
const DriverState freewheel_driver_state = {
    .motor_flags = enable_flags_none,
    .mode = DriverMode::GROUND_SHORT
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