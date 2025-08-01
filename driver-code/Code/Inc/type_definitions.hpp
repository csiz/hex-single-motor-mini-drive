#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

// Most useful datatype for this system, the 3 phase coordinates.
using ThreePhase = std::tuple<int, int, int>;

// Function signature for unit tests; the unit test prints text to the buffer.
using UnitTestFunction = void (*)(char * buffer, size_t max_size);


// Driver State
// ------------

enum struct DriverMode : uint16_t {
    OFF,
    FREEWHEEL,
    HOLD,
    SCHEDULE,
    DRIVE_6_SECTOR,
    DRIVE_PERIODIC,
    DRIVE_SMOOTH,
    DRIVE_TORQUE,
    DRIVE_BATTERY_POWER,
    SEEK_ANGLE_POWER,
    SEEK_ANGLE_TORQUE,
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
    uint16_t duration; // Duration in PWM cycles.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// Number of steps in test schedules.
const size_t schedule_size = 12;

// Motor driving PWM schedule.
using PWMSchedule = PWMStage[schedule_size];

// Pointer to a PWM schedule to run the test.
struct DriveSchedule {
    PWMSchedule const* pointer;
    uint16_t current_stage;
    uint16_t stage_counter;
};

// Drive the motor to a specific position.
struct SeekAngle {
    int16_t target_rotation;
    int16_t target_angle;
    int16_t max_secondary_target;
    int16_t secondary_target_residual;
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
    int16_t active_angle;
    
    // PWM value actively used to drive the motor.
    int16_t active_pwm;
    
    // Angular speed at which the active_angle is spinning.
    int16_t angular_speed;
    
    // The angle fraction remaining. Needed because the angular speed has higher
    // resolution than the angle; so we need to keep track of partial increments.
    int16_t active_angle_residual;

    // The target PWM value that we are aiming for with the smooth control. This
    // target can be set by the advanced control algorithms.
    int16_t target_pwm;

    // The lead angle value used to adjust the angle of the driven phase
    // to obtain a current that leads the rotor magnetic orientation by 90 degrees.
    int16_t lead_angle;

    // Higher resolution control value for the target PWM.
    int32_t target_pwm_control;

    // Higher resolution control value for the lead angle.
    int32_t lead_angle_control;

    int16_t secondary_target;

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

// Response data structures
// ------------------------

struct Readout{
    // The PWM commands for the motor outputs; concatenated into a single value.
    uint32_t pwm_commands;
    // Readout number; used to identify the readout in the history.
    uint16_t readout_number;
    // Driver state flags; packed into a single 16-bit value.
    uint16_t state_flags;
    // Raw phase U current readout (ADC value).
    int16_t u_current;
    // Raw phase V current readout (ADC value).
    int16_t v_current;
    // Raw phase W current readout (ADC value).
    int16_t w_current;
    // Raw reference readout (ADC value); this is the reference voltage for the current 
    // readouts as seen by the amplifier. Needs to be subtracted from the phase readouts.
    int16_t ref_readout;
    // Phase U current readout difference to previous readout.
    int16_t u_current_diff;
    // Phase V current readout difference to previous readout.
    int16_t v_current_diff;
    // Phase W current readout difference to previous readout.
    int16_t w_current_diff;
    // Best estimate for the rotor magnetic angle.
    int16_t angle;
    // Error of the angle measured from EMF to the rotor angle prediction.
    int16_t angle_adjustment;
    // Best estimate for the rotor magnetic angular speed.
    int16_t angular_speed;
    // Instantaneous VCC voltage readout (ADC value); from resistance divider.
    int16_t vcc_voltage;
    // EMF voltage magnitude. The EMF is always along the beta direction, but we can have errors in the
    // measurements and the rotor position and thus we see alpha component as well. We can rotate the
    // EMF voltage vector fully to the beta direction and get closer to the actual EMF voltage magnitude.
    int16_t emf_voltage_magnitude;
};

struct FullReadout : public Readout {
    // Tick rate; the number of main loop (communication and commands) updates per second.
    uint16_t main_loop_rate;
    // Update rate for the control loop; run from the ADC interrupts, once per cycle, ideally 23.4KHz.
    uint16_t adc_update_rate;
    // Instantaneous temperature readout (ADC value); from the temperature sensor.
    uint16_t temperature;
    // Current maximum PWM allowed by the driver.
    uint16_t live_max_pwm;

    // PWM counter value at the start of the control update. Should occur 
    // immediately after the halfway point.
    int16_t cycle_start_tick;
    // PWM counter value at the end of the control update. This allows us
    // to measure the time it took to process the control update. It also
    // let's us know if we updated the motor outputs before the next cycle.
    int16_t cycle_end_tick;

    // Current in DQ0 coordinates; aligned with the rotor angle.
    int16_t alpha_current;
    // Current in DQ0 coordinates; crossed with the rotor angle.
    int16_t beta_current;
    // EMF voltage in DQ0 coordinates; aligned with the rotor angle.
    int16_t alpha_emf_voltage;
    // EMF voltage in DQ0 coordinates; crossed with the rotor angle.
    int16_t beta_emf_voltage;

    // Total power used/given to VCC line (the battery usually).
    int16_t total_power;
    // Resistive power; the power dissipated in the phase resistances.
    int16_t resistive_power;
    // EMF power; the power used to drive the motor (which is reflected to the inductors as back EMF).
    int16_t emf_power;
    // Inductive power; the power pushed into the inductor magnetic fields.
    int16_t inductive_power;

    // Motor constant; the ratio between the EMF voltage induced in the coils and the magnet angular speed.
    // 
    // Also the ratio between the torque produced by the magnetic rotor and the current 
    // induced in the inductor coils (aka. the acceleration of the inductor angle).
    // 
    // Also the ratio between the torque induced in the inductor coils (aka. the current 
    // dumped/tanken from the inductor coils by the accelerating magnetic rotor).
    // 
    // ... and finally the ratio between the flux linkage to the magnetic rotor from the
    // inductor coils and inductor angular speed.
    // 
    // All of the above statements are equivalent; because they relate the relative position
    // and velocity between the inductor coil magnetic field and the rotor magnetic field.
    int16_t motor_constant;

    // The current angle.
    int16_t inductor_angle;

    // The measured acceleration of the rotor.
    int16_t rotor_acceleration;

    // Integrated number of EMF deduced rotor angle rotations since startup.
    int16_t rotations;

    // Magnitude of the phase current in the DQ0 coordinate frame.
    int16_t current_magnitude;

    // Variance of the EMF angle error; used to determine if the EMF angle is too noisy to update.
    int16_t emf_angle_error_variance;

    // Lead angle for the motor driving; used to adjust the phase voltages to drive the motor efficiently.
    int16_t lead_angle;

    // Target PWM value for the motor outputs, value set by the advanced control algorithms.
    int16_t target_pwm;
};

// Parameters data structures
// --------------------------

// Hall sensor angles at the switching points between the sectors.
using TriggerAngles = std::array<std::array<uint16_t, 2>, 6>;
// Variances for the hall sensor angles at the switching points between the sectors.
using TriggerAngleVariances = std::array<std::array<uint16_t, 2>, 6>;
// Center angles of the Hall sensor sectors.
using CenterAngles = std::array<uint16_t, 6>;
// Variances for the center angles of the Hall sensor sectors.
using CenterVariances = std::array<uint16_t, 6>;

// Hall sensor position calibration data.
// 
// Apparently, millimiter precision in the placement of the hall sensor chips means an error up to 
// 30 degrees in the electrical angle of the magnetic rotor. Note that for each physical rotation
// of the magnet there are N magnet poles times P coil pairs rotations of the electrical angle.
// 
// With this big of an error, we need to calibrate the hall sensor positions using the angle
// inferred from the back EMF voltage induced in the coils.
struct PositionCalibration {

    // The angle at the transition to the current sector from the left and from the right.
    // 
    // By "left" I mean the hall sector has transitioned from a lower to a higher number,
    // the rotor has a positive speed and is rotating counter-clockwise (trigonometric direction).
    // The left angle is lower than the right angle.
    // 
    // Note that the left angle of a sector and the right angle of the previous sector do not
    // coincide because the hall sensors have a designed hysterisis that latches the output.
    TriggerAngles sector_transition_angles;

    // The variance of the angles (it is expensive to compute the standard deviation with 
    // a square root but we only need the variance, so we only store the variance).
    TriggerAngleVariances sector_transition_variances;

    // The center angle of each sector; the average of the left and right angles.
    CenterAngles sector_center_angles;

    // The variance of the center angles; at the moment it represents the span of the hall sector.
    CenterVariances sector_center_variances;
};

struct CurrentCalibration {
    int16_t u_factor;
    int16_t v_factor;
    int16_t w_factor;
    int16_t inductance_factor;
};

// Parameters used in the motor control loop.
struct ControlParameters {

    // Magnet position integral gain.
    int16_t rotor_angle_ki;

    // Magnet angular speed integral gain.
    int16_t rotor_angular_speed_ki;
        
    // Averaging gain for the acceleration of the rotor.
    int16_t rotor_acceleration_ki;
    
    // Motor constant integral gain.
    int16_t motor_constant_ki;
    

    // Phase resistance gain.
    int16_t resistance_ki;
    
    // Inductance gain.
    int16_t inductance_ki;
    
    // Maximum PWM adjustment per cycle.
    int16_t max_pwm_change;
    
    // Maximum target angle change per cycle.
    int16_t max_angle_change;
    

    // Minimum EMF voltage to consider EMF detected (above the noise level)
    int16_t min_emf_voltage;
    
    // Integral gain for the hall angle adjustment (0 to ignore).
    int16_t hall_angle_ki;

    // Lead angle integral gain for efficient driving.
    int16_t lead_angle_control_ki;

    // Torque control gain.
    int16_t torque_control_ki;


    // Battery power control gain.
    int16_t battery_power_control_ki;

    // Speed control gain.
    int16_t speed_control_ki;

    // Probing angular speed for initial EMF detection.
    int16_t probing_angular_speed;

    // Maximum probing PWM value.
    int16_t probing_max_pwm;


    // Maximum EMF angle correction variance when it's too noisy to update the angle.
    int16_t emf_angle_error_variance_threshold;

    // Minium EMF voltage to compute the motor constant.
    int16_t min_emf_for_motor_constant;

    int16_t max_resistive_power;

    int16_t resistive_power_ki;


    int16_t max_angular_speed;

    int16_t spare_1;

    int16_t seek_via_torque_ki;

    int16_t seek_via_torque_kp;
    

    int16_t seek_via_torque_kd;

    int16_t seek_via_power_ki;

    int16_t seek_via_power_kp;

    int16_t seek_via_power_kd;
};


// Command data structures
// -----------------------

struct BasicCommand {
    uint16_t timeout;
    int16_t value;
    int16_t second;
    int16_t third;
};



// Three phase helper functions
// ----------------------------


// Extract the three phase duty cycles from the motor outputs.
static inline ThreePhase get_duties(MotorOutputs const& outputs) {
    return ThreePhase{
        static_cast<int>(outputs.u_duty),
        static_cast<int>(outputs.v_duty),
        static_cast<int>(outputs.w_duty)
    };
}

// Extract the three phase currents from the readout.
static inline ThreePhase get_currents(FullReadout const& readout) {
    return ThreePhase{
        readout.u_current,
        readout.v_current,
        readout.w_current
    };
}

// Extract the three phase currents differences from the readout.
static inline ThreePhase get_currents_diff(FullReadout const& readout) {
    return ThreePhase{
        readout.u_current_diff,
        readout.v_current_diff,
        readout.w_current_diff
    };
}

// Grab the calibration factors as a three phase tuple.
static inline ThreePhase get_calibration_factors(CurrentCalibration const& calibration) {
    return ThreePhase{
        calibration.u_factor,
        calibration.v_factor,
        calibration.w_factor
    };
}


// Adjust the three-phase values so that their sum is zero.
static inline ThreePhase adjust_to_sum_zero(ThreePhase const& values) {
    // Adjust the values so that their sum is zero.
    const int avg = (std::get<0>(values) + std::get<1>(values) + std::get<2>(values)) / 3;
    return ThreePhase{
        std::get<0>(values) - avg,
        std::get<1>(values) - avg,
        std::get<2>(values) - avg
    };
}

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
static inline ThreePhase operator * (ThreePhase const& a, int b) {
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
static inline ThreePhase operator / (ThreePhase const& a, int b) {
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

// Dot product between three phase vectors.
static inline int dot(ThreePhase const& a, ThreePhase const& b) {
    return (
        std::get<0>(a) * std::get<0>(b) +
        std::get<1>(a) * std::get<1>(b) +
        std::get<2>(a) * std::get<2>(b)
    );
}