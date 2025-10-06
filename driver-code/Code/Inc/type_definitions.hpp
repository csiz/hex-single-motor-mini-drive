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

// Driving modes for the motor control loop.
enum struct DriverMode : uint16_t {
    // All motor outputs are set to 0, the coils are all connected to ground slowing down the motor.
    OFF,
    // Motor outputs are tri-state / floating. The motor coils are not connected to the driver and the
    // motor is free to spin. Note that at large speeds the motor will generate a back EMF that can
    // overcome the MOSFETs body diodes and effectively turn the MOSFETs on. At that point the motor
    // will break, behaving as in the OFF mode. The only way to prevent it is to disconnect the cable.
    FREEWHEEL,
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
    uint16_t duration;
    // PWM duty cycle for U phase.
    uint16_t u_duty;
    // PWM duty cycle for V phase.
    uint16_t v_duty;
    // PWM duty cycle for W phase.
    uint16_t w_duty;
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
    // The target rotation index we want the motor to drive towards.
    int16_t target_rotation;
    // The target angle. This is the fractional part of the target rotation.
    int16_t target_angle;
    // The maximum control value (torque, power or speed) used to drive the motor to the target angle.
    int16_t max_secondary_target;
    // The high resolution integral error for the PID control loop.
    int32_t error_integral;
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
    int16_t direct_current;
    // Current in DQ0 coordinates; crossed with the rotor angle.
    int16_t quadrature_current;
    // EMF voltage in DQ0 coordinates; aligned with the rotor angle.
    int16_t direct_emf_voltage;
    // EMF voltage in DQ0 coordinates; crossed with the rotor angle.
    int16_t quadrature_emf_voltage;

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

    // Target for the advanced control algorithms.
    int16_t secondary_target;

    // Spare debug output.
    int16_t seek_integral;
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

// Calibration factors for the current sensors.
//
// The soldering joints vary during manufacturing and therefore they affect the total
// resistance of the shunt resistors. We can calibrate for this effect by multiplying
// the readout by a factor for each phase.
// 
// For the v1 design, we shall improve the shunt resistors and soldering pad design to
// to improve the accuracy. We can then switch to automatically calibrating the phase
// resistance and motor inductance. For now we calibrate using the motor monitor app.
struct CurrentCalibration {
    // Adjustment factor for the U phase current readout.
    int16_t u_factor;
    // Adjustment factor for the V phase current readout.
    int16_t v_factor;
    // Adjustment factor for the W phase current readout.
    int16_t w_factor;
    // Adjustment factor for the motor inductance; used to calibrate the coil inductance.
    int16_t inductance_factor;
};

// Parameters used in the motor control loop; for detailed descriptions check the
// motor monitor page. It's useful to modify the values and inspect the changes to
// the respective variables in the readout while driving a physical motor.
struct ControlParameters {

    // Magnet position integral gain.
    int16_t rotor_angle_ki;

    // Magnet angular speed integral gain.
    int16_t rotor_angular_speed_ki;
        
    // Averaging gain for the acceleration of the rotor.
    int16_t rotor_acceleration_ki;
    
    // Motor constant integral gain.
    int16_t motor_constant_ki;
    

    // Sign of the motor direction (positive by default, negative to reverse turning direction).
    int16_t motor_direction;

    // Number of incorrect direction detections before we flip our motor angle.
    int16_t incorrect_direction_threshold;
    
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

    // Maximum PWM difference from motor PWM required to compensate back EMF.
    int16_t max_pwm_difference;


    // Maximum EMF angle correction variance when it's too noisy to update the angle.
    int16_t emf_angle_error_variance_threshold;

    // Minium EMF voltage to compute the motor constant.
    int16_t min_emf_for_motor_constant;

    // Maximum resistive power that can be dissipated in the motor coils.
    int16_t max_resistive_power;

    // Resistive power long duration average observer gain.
    int16_t resistive_power_ki;


    // Maximum angular speed of the motor.
    int16_t max_angular_speed;

    // Maximum power draw from the battery (proxy for maximum current).
    int16_t max_power_draw;

    // Power draw long duration average observer gain.
    int16_t power_draw_ki;

    // Maximum PWM value for the motor outputs.
    int16_t max_pwm;
    
    // Seek via torque, prediction duration factor for integral error.
    int16_t seek_via_torque_k_prediction;
    // Seek via torque, integral gain for the PID control.
    int16_t seek_via_torque_ki;
    // Seek via torque, proportional gain for the PID control.
    int16_t seek_via_torque_kp;
    // Seek via torque, derivative gain for the PID control.
    int16_t seek_via_torque_kd;


    // Seek via power, prediction duration factor for integral error.
    int16_t seek_via_power_k_prediction;
    // Seek via power, integral gain for the PID control.
    int16_t seek_via_power_ki;
    // Seek via power, proportional gain for the PID control.
    int16_t seek_via_power_kp;
    // Seek via power, derivative gain for the PID control.
    int16_t seek_via_power_kd;

    // Seek via speed, prediction duration factor for integral error.
    int16_t seek_via_speed_k_prediction;
    // Seek via speed, integral gain for the PID control.
    int16_t seek_via_speed_ki;
    // Seek via speed, proportional gain for the PID control.
    int16_t seek_via_speed_kp;
    // Seek via speed, derivative gain for the PID control.
    int16_t seek_via_speed_kd;

    // Resistance of motor coils per phase (star configuration).
    int16_t phase_resistance;

    // Inductance of motor coils per phase (star configuration).
    int16_t phase_inductance;
};


// Command data structures
// -----------------------

// There is a minimum packed size for each message, so all messages (commands from
// the perspective of driver) will contain a minimal amount of data. Most driving
// commands will only use these fields to keep the command size small. Additional
// parameters can be set through the control parameters structure.
struct BasicCommand {
    // Duration to run each command in PWM cycles. Note that all driving commands
    // are timed and the motor turns off when it loses contact with the controller.
    uint16_t timeout;
    // The main command value.
    int16_t value;
    // Additional value depending on the command type.
    int16_t second;
    // Third value depending on the command type.
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