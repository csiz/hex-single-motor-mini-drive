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

struct PWMStage {
    // Duration of the stage in PWM cycles.
    uint32_t duration;
    // PWM duty cycle for U phase.
    float u_duty;
    // PWM duty cycle for V phase.
    float v_duty;
    // PWM duty cycle for W phase.
    float w_duty;
};

// Number of steps in test schedules.
const size_t schedule_size = 12;

// Motor driving PWM schedule.
using PWMSchedule = PWMStage[schedule_size];

// Pointer to a PWM schedule to run the test.
struct DriveSchedule {
    // Pointer to the schedule data.
    PWMSchedule const* pointer;
    // Current stage in the schedule.
    uint16_t current_stage;
    // Counter for the time spent (number of cycles) in the current stage.
    uint16_t stage_counter;
};

// Drive the motor to a specific position.
struct SeekAngle {
    // The target angle. This is the fractional part of the target rotation.
    int32_t target_angle;
    // The target rotation index we want the motor to drive towards.
    int32_t target_rotation;
};

struct TestParameters {
    float test_speed;
    uint32_t test_duration;
    int32_t test_angle;
};

struct DriverCommand {
    // Duration for the command in PWM cycles.
    uint16_t duration;
    // Control flags for the command.
    uint8_t motor_flags;
    // The mode of the driver for this command.
    DriverMode mode;
    // Start angle for the command.
    int32_t angle;
    // The angular speed for the command.
    float angular_speed;
    // Start value for the PWM drive.
    float pwm_value;
    // Incremental change for the PWM drive.
    float pwm_change;
};

// The complete driver state, these values control the motor behaviour.
struct DriverState {
    // Settings for the 3 phase MOSFET PWM drivers.
    MotorOutputs motor_outputs = breaking_motor_outputs;

    uint8_t motor_enable_flags = enable_flags_all;

    // Motor driving mode (defaults to short circuit breaking).
    DriverMode mode;

    // Duration for the command in pwm cycles.
    uint16_t duration;

    // Angle at which the motor is currently driven.
    int32_t active_angle;
    
    // The angular speed of the driving wave.
    float active_angular_speed;

    // PWM value actively used to drive the motor.
    float active_pwm;

    // PWM value that we're aiming for in the tight loop with smooth control.
    float target_pwm;

    // The lead angle value used to adjust the angle of the driven phase
    // to obtain a current that leads the rotor magnetic orientation by 90 degrees.
    int32_t lead_angle;
};

// Driver state for safely breaking the motor.
const DriverState breaking_driver_state = {
    .motor_outputs = breaking_motor_outputs,
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