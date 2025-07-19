#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

// Most useful datatype for this system, the 3 phase coordinates.
using ThreePhase = std::tuple<int, int, int>;

// Driver State
// ------------

enum struct DriverState : uint16_t {
    NO_CHANGE,
    OFF,
    FREEWHEEL,
    HOLD,
    SCHEDULE,
    DRIVE_6_SECTOR,
    DRIVE_PERIODIC,
    DRIVE_SMOOTH,
    DRIVE_TORQUE,
    DRIVE_BATTERY_POWER
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

struct PWMStage {
    uint16_t duration; // Duration in PWM cycles.
    uint16_t u_duty; // PWM duty cycle for U phase.
    uint16_t v_duty; // PWM duty cycle for V phase.
    uint16_t w_duty; // PWM duty cycle for W phase.
};

// The hold command is the same as the motor outputs.
using DriveHold = PWMStage;

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

// Drive motor using the 6 sector commutation method.
struct Drive6Sector {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t pwm_target;
};

struct DrivePeriodic {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t zero_offset; // Initial angle at readout number 0.
    int16_t pwm_target; // Target PWM value.
    int16_t angular_speed; // Angular speed for the drive command.
};

// Drive the motor using FOC targeting a PWM value.
struct DriveSmooth {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t zero_offset;
    int16_t pwm_target;
    int16_t pwm_active;
    int16_t lead_angle_control;
};

// Drive the motor to a specific current target (torque target).
struct DriveTorque {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t zero_offset;
    int16_t current_target; // Target current in fixed point format.
    int16_t torque_control;
};

// Drive the motor to a specific battery power drain.
struct DriveBatteryPower {
    uint16_t duration; // Duration for the command in pwm cycles.
    int16_t zero_offset;
    int16_t power_target; // Target power in fixed point format.
    int16_t battery_power_control;
};

// Drive parameters for each state.
union DriverParameters {
    DriveHold hold;
    DriveSchedule schedule;
    Drive6Sector sector;
    DrivePeriodic periodic;
    DriveSmooth smooth;
    DriveTorque torque;
    DriveBatteryPower battery_power;
};

const DriverParameters null_driver_parameters = {};


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
    int16_t u_readout;
    // Raw phase V current readout (ADC value).
    int16_t v_readout;
    // Raw phase W current readout (ADC value).
    int16_t w_readout;
    // Raw reference readout (ADC value); this is the reference voltage for the current 
    // readouts as seen by the amplifier. Needs to be subtracted from the phase readouts.
    int16_t ref_readout;
    // Phase U current readout difference to previous readout.
    int16_t u_readout_diff;
    // Phase V current readout difference to previous readout.
    int16_t v_readout_diff;
    // Phase W current readout difference to previous readout.
    int16_t w_readout_diff;
    // Best estimate for the rotor magnetic angle.
    int16_t angle;
    // Best estimate for the rotor magnetic angular speed.
    int16_t angular_speed;
    // Instantaneous VCC voltage readout (ADC value); from resistance divider.
    int16_t instant_vcc_voltage;
};

struct FullReadout : public Readout {
    // Tick rate; the number of main loop (communication and commands) updates per second.
    uint16_t main_loop_rate;
    // Update rate for the control loop; run from the ADC interrupts, once per cycle, ideally 23.4KHz.
    uint16_t adc_update_rate;
    // Instantaneous temperature readout (ADC value); from the temperature sensor.
    uint16_t temperature;
    // Average VCC voltage; smoothed out over some cycles.
    uint16_t vcc_voltage;

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
    // EMF voltage magnitude. The EMF is always along the beta direction, but we can have errors in the
    // measurements and the rotor position and thus we see alpha component as well. We can rotate the
    // EMF voltage vector fully to the beta direction and get closer to the actual EMF voltage magnitude.
    int16_t emf_voltage_magnitude;
    // The measured acceleration of the rotor.
    int16_t rotor_acceleration;
    
    // Error of the angle measured from EMF to the rotor angle prediction.
    int16_t angle_error;

    // The phase resistance; the drag factor for the fixed reference frame current.
    int16_t phase_resistance;

    // The phase inductance; behaves as if the inductor coil magnetic field had a mass. It's 
    // smaller than the rotor mass. It controls the ratio between the magnet acceleration
    // and the (reversely felt) inductor acceleration. Also the classical L/R time constant
    // for the fixed reference frame current.
    int16_t phase_inductance;

    int16_t u_debug;
    int16_t v_debug;
    int16_t w_debug;
};


// Command data structures
// -----------------------

struct BasicCommand {
    uint16_t code;
    uint16_t timeout;
    int16_t value;
    int16_t secondary;
};

struct CurrentCalibration {
    int16_t u_factor;
    int16_t v_factor;
    int16_t w_factor;
    int16_t inductance_factor;
};


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
    
    // Minimum speed when we are certain to detect the EMF voltage.
    int16_t min_emf_speed;

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
};
