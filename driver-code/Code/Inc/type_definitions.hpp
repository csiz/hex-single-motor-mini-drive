#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

#include "hex_mini_drive_interface.hpp"

// Most useful datatype for this system, the 3 phase coordinates.
using ThreePhase = std::tuple<float, float, float>;

// Function signature for unit tests; the unit test prints text to the buffer.
using UnitTestFunction = void (*)(char * buffer, size_t max_size);

// ADC Readings
// ------------

struct ADCReadings {
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
    uint16_t temp_readout;
    uint16_t vcc_readout;
};


// Driver State
// ------------

// Driving modes for the motor control loop.
enum struct DriverMode : uint16_t {
    // All motor outputs are set to 0, the coils are all connected to ground slowing down the motor.
    OFF,
    // Motor outputs are tri-state / floating. The motor coils are not connected to the driver and the
    // motor is free to spin. Note that at large speeds the motor will generate a back EMF that can
    // overcome the MOSFETs body diodes and effectively turn the MOSFETs on. At that point the motor
    // will break, behaving as in the OFF mode. The only way to prevent it is to disconnect the cable.
    FREEWHEEL,
    // Continue previous command, used to ping the interrupt loop.
    CONTINUE,
    // Hold the commanded duty cycle for each phase.
    HOLD,
    // Drive the motor using a time indexed schedule with a sequence of hold commands.
    SCHEDULE,
    // Drive the motor using 6 sector driving. This is the usual drive mode for BLDC motor speed controllers (which are not FOC).
    DRIVE_6_SECTOR,
    // Drive the motor with an open loop periodic excitation of the motor coils. The phases are
    // driven around a circle at the specified PWM and speed.
    DRIVE_PERIODIC,
    // Drive the motor using FOC targeting a PWM value. The active driving PWM and the active angle are
    // varied smoothly and target a current that is as close to 90 degrees ahead of the magnetic angle so
    // that we maximize the driving efficiency.
    DRIVE_SMOOTH,
    // Drive the motor using a fast, integral only, PID loop to control the torque (coil current) produced by the motor.
    DRIVE_TORQUE,
    // Drive the motor using a fast, integral only, PID loop to control the battery power (battery current) consumed by the motor.
    DRIVE_BATTERY_POWER,
    // Drive the motor using a fast, integral only, PID loop to control the speed of the motor.
    DRIVE_SPEED,
    // Drive the motor to a specific angle. Uses a secondary PID loop to control the power consumed 
    // by the motor to achieve the target angle.
    SEEK_ANGLE_POWER,
    // Drive the motor to a specific angle. Uses a secondary PID loop to control the torque produced
    // by the motor to achieve the target angle. Without the integral or derivative term this drive
    // mode will make the motor behave like a spring.
    SEEK_ANGLE_TORQUE,
    // Drive the motor to a specific angle. Uses a secondary PID loop to control the speed of the motor.
    SEEK_ANGLE_SPEED,
    // Drive the motor phases with a pyramid waveform at each phase angle to measure the current
    // response and determine the resistance of the motor windings.
    RESISTANCE_CALIBRATION,
    // Drive the motor phases with large steps at opposite poles to measure the current response
    // and determine the inductance of the motor windings.
    INDUCTANCE_CALIBRATION,
};

// Motor duty cycle (compare register values and enable settings).
struct MotorOutputs {
    uint16_t enable_flags; // Flags to enable/disable the motor outputs.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// Enable all motor outputs (U, V, W).
const uint16_t enable_flags_all = 0b111;
// Enable U phase outputs.
const uint16_t enable_flags_u = 0b001;
// Enable V phase outputs.
const uint16_t enable_flags_v = 0b010;
// Enable W phase outputs.
const uint16_t enable_flags_w = 0b100;
// Disable all motor outputs.
const uint16_t enable_flags_none = 0b000;

// Zeroed motor outputs, used to short circuit break the motor outputs.
const MotorOutputs breaking_motor_outputs = {
    .enable_flags = enable_flags_all,
    .u_duty = 0,
    .v_duty = 0,
    .w_duty = 0
};

// Freewheel motor outputs, used to disconnect the motor phases.
const MotorOutputs freewheel_motor_outputs = {
    .enable_flags = enable_flags_none,
    .u_duty = 0,
    .v_duty = 0,
    .w_duty = 0
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
    // The maximum control value (torque, power or speed) used to drive the motor to the target angle.
    float max_secondary_target;
    // The high resolution integral error for the PID control loop.
    float error_integral;
};

// The complete driver state, these values control the motor behaviour.
struct DriverState {
    // Settings for the 3 phase MOSFET PWM drivers.
    MotorOutputs motor_outputs = breaking_motor_outputs;

    // Motor driving mode (defaults to short circuit breaking).
    DriverMode mode;

    // Duration for the command in pwm cycles.
    uint16_t duration;

    // Angle at which the motor is currently driven.
    int32_t active_angle;
    
    // PWM value actively used to drive the motor.
    float active_pwm;
    
    // Angular speed at which the active_angle is spinning.
    float angular_speed;
    
    // The target PWM value that we are aiming for with the smooth control. This
    // target can be set by the advanced control algorithms.
    float target_pwm;

    // The lead angle value used to adjust the angle of the driven phase
    // to obtain a current that leads the rotor magnetic orientation by 90 degrees.
    int32_t lead_angle;

    float secondary_target;

    // The additional data depends on the driver mode.
    union {
        // Drive the motor using a fixed PWM schedule.
        DriveSchedule schedule;

        // Drive the motor to a specific angle.
        SeekAngle seek_angle;
    };
};

// Driver state for safely breaking the motor.
const DriverState breaking_driver_state = {
    .motor_outputs = breaking_motor_outputs,
    .mode = DriverMode::OFF
};

// Size of the DriverState structure (we want to keep it small for performance).
const size_t driver_state_size = sizeof(DriverState);

static_assert(driver_state_size <= 64, "DriverState size exceeds 64 bytes, try to make it smaller!");


// Three phase helper functions
// ----------------------------


// Extract the three phase currents from the readout.
static inline ThreePhase get_currents(hex_mini_drive::FullReadout const& readout) {
    return ThreePhase{
        readout.u_current,
        readout.v_current,
        readout.w_current
    };
}

// Extract the three phase currents differences from the readout.
static inline ThreePhase get_currents_diff(hex_mini_drive::FullReadout const& readout) {
    return ThreePhase{
        readout.u_current_diff,
        readout.v_current_diff,
        readout.w_current_diff
    };
}

// Grab the calibration factors as a three phase tuple.
static inline ThreePhase get_phase_resistances(hex_mini_drive::CurrentCalibration const& calibration) {
    return ThreePhase{
        calibration.phase_u_resistance,
        calibration.phase_v_resistance,
        calibration.phase_w_resistance,
    };
}

const float three_inverse = 1.0 / 3.0;

// Adjust the three-phase values so that their sum is zero.
static inline ThreePhase adjust_to_sum_zero(ThreePhase const& values) {
    // Adjust the values so that their sum is zero.
    const float avg = (std::get<0>(values) + std::get<1>(values) + std::get<2>(values)) * three_inverse;
    return ThreePhase{
        std::get<0>(values) - avg,
        std::get<1>(values) - avg,
        std::get<2>(values) - avg
    };
}

// Create a three phase tuple with the same value for all phases.
static inline ThreePhase three_same(float value) {
    return ThreePhase{value, value, value};
}

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


