#include "interrupts.hpp"
#include "interrupts_pid.hpp"

#include <cstdint>

#include "hex_mini_drive_interface.hpp"

#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "io.hpp"
#include "constants.hpp"
#include "type_definitions.hpp"

// The interrupts must not enter the error handler!
// 
// The error handle will block forever, however we must safe the motor no matter what. The interrupt loop
// will always timeout any command and return to a safe state if it doesn't receive new commands from the
// main app loop.
// 
// Do not: #include "error_handler.hpp"


#include <stm32g4xx_ll_adc.h>
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_cordic.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <bit>
#include <tuple>

#include "hex_mini_drive_interface.hpp"

// Interrupt loop
// ==============

// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.


// Interrupt Loop State
// --------------------

// Electrical and position state
hex_mini_drive::FullReadout readout = {
    .live_max_pwm = pwm_max,
    .emf_angle_error_variance = square(quarter_circle),
};

// Latest readout we have copied from the shared_readout in the main loop.
hex_mini_drive::FullReadout latest_readout = readout;

// The readout where the ADC loop writes the latest value if not locked by the main loop.
hex_mini_drive::FullReadout shared_readout = readout;

// Lock away writes from the ADC loop so we can copy the shared_readout to the main loop.
volatile bool shared_readout_lock = false;

// History of light readouts so that we can record every cycle for a short snapshot.
hex_mini_drive::Readout readout_history[hex_mini_drive::HISTORY_SIZE] = {};

// Current write index.
volatile size_t readout_history_write_index = 0;

// Mark the readout history for reset.
volatile bool readout_history_reset_flag = false;

// Angle offsetting from the main loop or via commands; it will not influence speed.
volatile int32_t external_angle_offset = 0;

// Additional state
// ----------------


// Track how many times we think our angle is correct.
int32_t correct_angle_counter = 0;

const int32_t angle_fix_threshold_count = 16;
const int32_t angle_fix_max_count = 512;

// Our outputs are delayed 1 cycle; store the previous outputs here before we use them.
ThreePhase previous_half_cycle_drive_voltages = {0, 0, 0};

// Motor driver state
// ------------------

// Currently active driver state (the full motor control state should be stored here).
DriverState driver_state = breaking_driver_state;

// Must be volatile as it's the interaction flag between main loop and the interrupt handler.
// 
// The main loop will set pending_state and pending_state to the user command, but only
// when the pending_state is DriverState::NO_CHANGE. The interrupt handler will copy the
// pending_state and pending_state to the active state variables, and reset the pending_state.
// 
// Volatile will prevent the compiler from optimizing out the read/write operations to this variable.
// In our case, the main loop will read this variable in a hot while loop that has no side effects, 
// expecting the variable to be set by the interrupt handler. The compiler *will* optimize out the
// while loop unless we mark the variable as volatile.
volatile bool new_pending_state = false;

// Settings for the new driver state.
DriverState pending_state = breaking_driver_state;



// Interrupt Data Interface
// ------------------------


// Initialize the loop control parameters and the calibration data. Either load 
// them from the flash or use the defaults.

hex_mini_drive::CurrentCalibration current_calibration = get_current_calibration();
hex_mini_drive::ControlParameters control_parameters = get_control_parameters();

// Guard the data access by indicating to the ADC interrupt that it shouldn't write data.
hex_mini_drive::FullReadout get_readout(){
    if (shared_readout_lock) {
        latest_readout = shared_readout;
        shared_readout_lock = false;
    }
    return latest_readout;
}

void readout_history_mark_reset() {
    readout_history_reset_flag = true;
}

bool readout_history_get_reset_flag() {
    return readout_history_reset_flag;
}

hex_mini_drive::Readout const* get_readout_history(){
    return readout_history;
}

size_t get_readout_history_size() {
    return readout_history_write_index;
}

// (Private func) Push a readout to the history buffer.
static inline bool readout_history_push(hex_mini_drive::Readout const& readout){
    if (readout_history_write_index >= hex_mini_drive::HISTORY_SIZE) return false;
    readout_history[readout_history_write_index] = readout;
    // Increment after we have finished copying the readout.
    readout_history_write_index += 1;
    return true;
}

// (Private func) Reset the readout history.
static inline void readout_history_reset() {
    readout_history_write_index = 0;
    readout_history_reset_flag = false;
}

bool is_motor_safed(){
    // Consider both motor breaking and freewheeling as safe states.
    return (driver_state.mode == DriverMode::OFF) || (driver_state.mode == DriverMode::FREEWHEEL);
}

void set_motor_command(DriverState const& driver_state){
    // Don't override a pending command if the interrupt loop didn't copy it to active.
    while (new_pending_state) continue;

    // Copy the commanded state to the pending queue.
    pending_state = driver_state;

    // Flag that we have a new command to process.
    new_pending_state = true;
}

void set_angle_offset(int32_t angle_offset) {
    external_angle_offset = angle_offset;
}

// Helper functions
// ----------------

static inline void set_cordic(int32_t const& x, int32_t const& y){
    LL_CORDIC_WriteData(CORDIC, std::bit_cast<uint32_t>(x));
    LL_CORDIC_WriteData(CORDIC, std::bit_cast<uint32_t>(y));
}

static inline std::pair<int32_t, int32_t> get_cordic(){
    return {
        std::bit_cast<int32_t>(LL_CORDIC_ReadData(CORDIC)),
        std::bit_cast<int32_t>(LL_CORDIC_ReadData(CORDIC))
    };
}


// Critical functions!! 23KHz PWM cycle
// ====================================

// Motor control functions
// -----------------------


// Update the motor outputs using simple 6 sector driving based on the hall sensors.
static inline MotorOutputs update_motor_6_sector(
    DriverState const& driver_state,
    hex_mini_drive::FullReadout const& readout
){

    // Update the sector variable.
    const uint8_t hall_sector = get_hall_sector(readout.state_flags & hall_state_bit_mask);

    // Check if the magnet is present.
    const bool angle_valid = hall_sector < hall_sector_base;

    if (not angle_valid) return breaking_motor_outputs;

    auto const& motor_sector_driving_table = driver_state.active_pwm >= 0 ? 
        motor_sector_driving_positive : 
        motor_sector_driving_negative;

    // Get the voltage for the three phases from the table.

    const float voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const float voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const float voltage_phase_w = motor_sector_driving_table[hall_sector][2];

    const float abs_pwm = min(
        readout.live_max_pwm,
        faster_abs(driver_state.active_pwm)
    );

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm)
    };
}

// Set the motor outputs to the specified active_pwm and active_angle.
static inline MotorOutputs update_motor_at_angle(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
) {
    // We need a single comparison and we can then use arithmetic math instead of branching conditionals.
    const bool negative_pwm = driver_state.active_pwm < 0.f;

    // The PWM counter value must be positive, we use the sign to determine the direction.
    const float abs_pwm = min(readout.live_max_pwm, negative_pwm ? -driver_state.active_pwm : driver_state.active_pwm);

    // Use the active angle or flip it depending on the sign of the PWM.
    const int32_t angle = driver_state.active_angle + negative_pwm * half_circle;

    // Get the voltage for the three phases from the waveform table.
    const float voltage_phase_u = get_phase_pwm(angle);
    const float voltage_phase_v = get_phase_pwm(angle - third_circle);
    const float voltage_phase_w = get_phase_pwm(angle - neg_third_circle);

    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm)
    };
}

// Drive the inductors around a circle at the specified PWM and speed (open loop control).
static inline MotorOutputs update_motor_periodic(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    driver_state.active_angle += static_cast<int32_t>(driver_state.target);

    return update_motor_at_angle(driver_state, readout);
}


// For the resistance calibration we will drive the motor U V and W phases with a pyramid waveform
// and measure the current response to determine the resistance of the motor windings.
static inline MotorOutputs update_motor_resistance_calibration(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // The total duration is HISTORY_SIZE, let's divide it into 6 segments where we
    // drive the current at 0, 60, 120, 180, 240, and 300 degrees phase angle. For
    // each phase divide the duration into 3 segments, first we stay at 0, then ramp
    // up to the target PWM, then ramp down to 0.

    // Number of PWM cycles elapsed since the calibration started.
    const int32_t elapsed = hex_mini_drive::HISTORY_SIZE - driver_state.duration;

    // Duration of a single phase angle segment.
    const int32_t segment_duration = hex_mini_drive::HISTORY_SIZE / 6;

    // Which of the 6 phase angle segments we are currently in.
    const int32_t segment_index = elapsed / segment_duration;

    // How far we are into the current segment.
    const int32_t segment_progress = elapsed - segment_index * segment_duration;

    // Duration of a single ramp (a third of the segment).
    const int32_t ramp_duration = segment_duration / 3;

    // Drive the phases at 0, 60, 120, 180, 240, and 300 degrees.
    driver_state.active_angle = static_cast<int32_t>(segment_index * (angle_base / 6));

    // Build a pyramid waveform: hold at 0, ramp up to the target PWM, then ramp back down to 0.
    const float abs_pwm = (
        segment_progress < ramp_duration ? 0 :
        segment_progress < 2 * ramp_duration ? driver_state.target * (segment_progress - ramp_duration) / ramp_duration :
        driver_state.target * (3.0 * ramp_duration - segment_progress) / ramp_duration
    );

    driver_state.active_pwm = clip_to(0.0f, driver_state.target, abs_pwm);

    return update_motor_at_angle(driver_state, readout);
}

// Drive the motor using large step increases to measure the inductance of the motor windings.
static inline MotorOutputs update_motor_inductance_calibration(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // For the inductance calibration we will also drive the 3 motor phases, however this time
    // we will do so in large steps and at opposite poles. This time split the HISTORY_SIZE total
    // into 3 segments, each phase is further divided into 6 segments as such, assuming we start at 0: 
    // first a step to half target PWM then a step to negative half target then step to 0,
    // then step to positive target PWM then a step to negative target PWM then back to 0.
    
    // Number of PWM cycles elapsed since the calibration started.
    const int elapsed = hex_mini_drive::HISTORY_SIZE - driver_state.duration;

    // Duration of a single phase angle segment (one per phase: 0, 120, and 240 degrees).
    const int segment_duration = hex_mini_drive::HISTORY_SIZE / 3;

    // Which of the 3 phase angle segments we are currently in.
    const int segment_index = elapsed / segment_duration;

    // How far we are into the current segment.
    const int segment_progress = elapsed - segment_index * segment_duration;

    // Duration of a single step (a sixth of the segment).
    const int step_duration = segment_duration / 6;

    // Which of the 6 steps we are currently in.
    const int step_index = segment_progress / step_duration;

    // Drive the phases at 0, 120, and 240 degrees.
    driver_state.active_angle = static_cast<int32_t>(segment_index * (angle_base * 0.33333333));

    // Half of the target PWM used for the first two steps.
    const int half_pwm = driver_state.target / 2;

    // Build the step waveform: +half, -half, 0, +full, -full, 0. Negative PWM drives the
    // opposite pole via update_motor_at_angle flipping the angle by half a circle.
    driver_state.active_pwm = (
        step_index == 0 ? +half_pwm :
        step_index == 1 ? -half_pwm :
        step_index == 3 ? +driver_state.target :
        step_index == 4 ? -driver_state.target :
        0
    );

    return update_motor_at_angle(driver_state, readout);
}

// The chirp test drives the inductor at pwm_target around the coils at the requested speed
// and number of rotations. Because the chirp is fast and the motor is presumed stationary,
// we can measure the residual voltage terms after accounting for resistance and inductance
// to isolate the response of the iron core of the coils. We can use this to determine the
// rotor position.
static inline MotorOutputs update_motor_position_calibration_chirp(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    if (driver_state.test_parameters.test_duration > 0) {
        driver_state.test_parameters.test_duration -= 1;

        // Drive at the requested PWM until the test is done.
        driver_state.active_angle += static_cast<int32_t>(driver_state.test_parameters.test_speed);
        driver_state.active_pwm = faster_abs(driver_state.target);
        return update_motor_at_angle(driver_state, readout);
    } else {
        return breaking_motor_outputs;
    }
}

// For the EMF test we spin the motor using start_speed and start_rotations, then we
// short the motor and start recording the EMF voltage.
static inline MotorOutputs update_motor_position_calibration_emf(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){

    if (driver_state.duration > hex_mini_drive::HISTORY_SIZE) {
        // Keep marking history for reset until we have finished spinning up the motor.
        readout_history_reset_flag = true;

        const int32_t elapsed = hex_mini_drive::HISTORY_SIZE + driver_state.test_parameters.test_duration - driver_state.duration;

        // Linearly ramp up the speed from 0 to test_speed over the duration of test_duration.
        const float speed = static_cast<float>(driver_state.test_parameters.test_speed) * static_cast<float>(elapsed) / static_cast<float>(driver_state.test_parameters.test_duration);


        // Spin up phase, spin the motor at start_speed and start_rotations.
        driver_state.active_angle += static_cast<int32_t>(speed);
        driver_state.active_pwm = faster_abs(driver_state.target);
        return update_motor_at_angle(driver_state, readout);

    } else {
        // Record the shorted outputs.
        return breaking_motor_outputs;
    }
}
    


// Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
// close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
// The PWM duty cycle, drive angle, and angle speed are varied smoothly.
static inline MotorOutputs update_motor_smooth(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // Check if we have an accurate readout angle.
    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;
    const bool current_detected = readout.state_flags & current_detected_bit_mask;

    // Base the direction on the sign of the target PWM.
    const int32_t active_pwm_sign = sign(driver_state.active_pwm);

    if (angle_fix) {
        // Drive towards the ideal angle; however decay to 0 at low EMF voltage.
        // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
        // 
        // Of course, the inductors take a while to charge and the rotor is producing an EMF
        // which all interacts with the current. However, the current that we end up measuring
        // should be as close to the 90 degrees as possible for maximum torque per current use.
        const int32_t ideal_angle = readout.angle + quarter_circle;
    
        // Get the error between the measured current and the ideal current angle.
        const int32_t lead_angle_error = current_detected * (ideal_angle - readout.current_angle);

        // Adjust the target angle to keep the alpha current small; reset if the motor is not moving.
        driver_state.lead_angle = clip_to(
            -max_lead_angle_control,
            +max_lead_angle_control,
            driver_state.lead_angle + static_cast<int32_t>(control_parameters.lead_angle_control_ki * lead_angle_error)
        );

        // Drive the motor to produce current perpendicular to the magnetic angle.
        driver_state.active_angle = ideal_angle + active_pwm_sign * driver_state.lead_angle;
    } else {
        // If we don't have an accurate position, we need drive the motor open loop until we get an EMF fix.

        // Use the probing speed.
        driver_state.active_angle += active_pwm_sign * static_cast<int32_t>(control_parameters.probing_angular_speed);
    }

    return update_motor_at_angle(driver_state, readout);
}

// Drive the motor with the desired output current (note the DQ0 current is 3/2 phase current).
// This mode also uses the smooth driving algorithm to drive using field oriented control, thus
// keeping the current as close to 90 degrees ahead of the magnetic angle as possible. Allowing
// control of the torque produced by the motor which will be proportional to the current target.
static inline MotorOutputs update_motor_torque(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // Squash very low currents to 0 to avoid noise.
    const bool current_detected = readout.state_flags & current_detected_bit_mask;

    // Get the signed current magnitude to compare against the target.
    const float measured_current = current_detected * sign(readout.quadrature_current) * readout.current_magnitude;

    // Calculate the difference between the target and measured current.
    const float control_error = (driver_state.target - measured_current) * max_drive_current_inverse;

    // Update the PID control for the torque.
    driver_state.active_pwm = clip_to(
        -pwm_max,
        +pwm_max,
        driver_state.active_pwm + control_error * control_parameters.torque_control_ki
    );

    return update_motor_smooth(driver_state, readout);
}

// Drive motor using up to a target battery power consumption.
// 
// The sign of the target power determines the direction of driving. When motor breaking, we
// try to absorb the target power instead and use it to charge the battery.
static inline MotorOutputs update_motor_battery_power(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // Note that total power will be 0 when not driving; in that case we want to
    // counter the EMF voltage to minimize phase resistance heating, but we don't
    // want to absorb more power than the target setting.

    const bool total_power_dominates = faster_abs(readout.total_power) > faster_abs(readout.emf_power);

    const float measured_power = (total_power_dominates ? 
        sign(driver_state.active_pwm) * readout.total_power :
        -sign(readout.quadrature_emf_voltage) * readout.emf_power
    );

    const float control_error = (driver_state.target - measured_power) * max_drive_power_inverse;


    // Update the PID control for the torque.
    driver_state.active_pwm = clip_to(
        -pwm_max,
        +pwm_max,
        driver_state.active_pwm + control_error * control_parameters.battery_power_control_ki
    );

    return update_motor_smooth(driver_state, readout);
}


static inline MotorOutputs update_motor_speed(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // Calculate the difference between the target and measured current.
    const float control_error = (driver_state.target - readout.angular_speed) * max_angular_speed_inverse;

    // Update the PID control for the torque.
    driver_state.active_pwm = clip_to(
        -pwm_max,
        +pwm_max,
        driver_state.active_pwm + control_error * control_parameters.speed_control_ki
    );

    return update_motor_smooth(driver_state, readout);
}


static inline MotorOutputs update_motor_seek_angle_power(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    const float pid_control = compute_seek_pid_control(
        driver_state.seek_angle,
        readout,
        control_parameters.seek_via_power_k_prediction,
        control_parameters.seek_via_power_ki,
        control_parameters.seek_via_power_kp,
        control_parameters.seek_via_power_kd
    );

    driver_state.target = driver_state.seek_angle.max_target * pid_control;

    return update_motor_battery_power(driver_state, readout);
}

static inline MotorOutputs update_motor_seek_angle_torque(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    const float pid_control = compute_seek_pid_control(
        driver_state.seek_angle,
        readout,
        control_parameters.seek_via_torque_k_prediction,
        control_parameters.seek_via_torque_ki,
        control_parameters.seek_via_torque_kp,
        control_parameters.seek_via_torque_kd
    );

    driver_state.target = driver_state.seek_angle.max_target * pid_control;

    return update_motor_torque(driver_state, readout);
}

static inline MotorOutputs update_motor_seek_angle_speed(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    const float pid_control = compute_seek_pid_control(
        driver_state.seek_angle,
        readout,
        control_parameters.seek_via_speed_k_prediction,
        control_parameters.seek_via_speed_ki,
        control_parameters.seek_via_speed_kp,
        control_parameters.seek_via_speed_kd
    );

    driver_state.target = driver_state.seek_angle.max_target * pid_control;

    return update_motor_speed(driver_state, readout);
}

// Drive the motor using a fixed schedule for the PWM outputs.
static inline MotorOutputs update_motor_schedule(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    PWMSchedule const& schedule = *driver_state.schedule.pointer;
    PWMStage const& schedule_stage = schedule[driver_state.schedule.current_stage];

    driver_state.schedule.stage_counter += 1;

    if (driver_state.schedule.stage_counter >= schedule_stage.duration) {
        // Move to the next stage in the schedule.
        driver_state.schedule.current_stage += 1;
        driver_state.schedule.stage_counter = 0;
    }
    
    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>(schedule_stage.u_duty * driver_state.target),
        .v_duty = static_cast<uint16_t>(schedule_stage.v_duty * driver_state.target),
        .w_duty = static_cast<uint16_t>(schedule_stage.w_duty * driver_state.target)
    };
}


// Motor control
// -------------

// Set the driver state to OFF.
static inline void set_breaking_control(DriverState & driver_state){
    driver_state = breaking_driver_state;
}

// Copy the pending driver state with all values clamped to valid ranges.
static inline DriverState setup_driver_state(
    DriverState const& driver_state,
    DriverState const& pending_state,
    hex_mini_drive::FullReadout const& readout
){
    switch(pending_state.mode){
        case DriverMode::OFF:
            return DriverState{
                .motor_outputs = breaking_motor_outputs,
                .mode = DriverMode::OFF
            };

        case DriverMode::FREEWHEEL:
            return DriverState{
                .motor_outputs = MotorOutputs{.enable_flags = enable_flags_none},
                .mode = DriverMode::FREEWHEEL
            };

        case DriverMode::CONTINUE:
            return driver_state;

        case DriverMode::HOLD:
            return DriverState{
                .motor_outputs = MotorOutputs {
                    .enable_flags = pending_state.motor_outputs.enable_flags,
                    .u_duty = static_cast<uint16_t>(clip_to(0, control_parameters.max_pwm_difference, pending_state.motor_outputs.u_duty)),
                    .v_duty = static_cast<uint16_t>(clip_to(0, control_parameters.max_pwm_difference, pending_state.motor_outputs.v_duty)),
                    .w_duty = static_cast<uint16_t>(clip_to(0, control_parameters.max_pwm_difference, pending_state.motor_outputs.w_duty))
                },
                .mode = DriverMode::HOLD,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
            };

        case DriverMode::SCHEDULE:
            // We should not enter testing mode without a valid schedule.
            return pending_state.schedule.pointer == nullptr ? breaking_driver_state : DriverState{
                .mode = DriverMode::SCHEDULE,
                .duration = hex_mini_drive::HISTORY_SIZE,
                .target = clip_to(0.0f, pwm_max, pending_state.target),
                .schedule = DriveSchedule{
                    .pointer = pending_state.schedule.pointer,
                    .current_stage = 0,
                    .stage_counter = 0
                }
            };
        
        case DriverMode::DRIVE_6_SECTOR:
            return DriverState{
                .mode = DriverMode::DRIVE_6_SECTOR,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_pwm = clip_to(-pwm_max, pwm_max, pending_state.active_pwm),
            };

        case DriverMode::DRIVE_PERIODIC:
            return DriverState{
                .mode = DriverMode::DRIVE_PERIODIC,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = pending_state.active_angle + (pending_state.active_pwm < 0 ? half_circle : 0),
                .active_pwm = min(control_parameters.max_pwm_difference, faster_abs(pending_state.active_pwm)),
                .target = clip_to(-max_angular_speed, max_angular_speed, pending_state.target),
            };

        case DriverMode::DRIVE_SMOOTH:
            // Maintain a some of the previous state so we can smoothly transition to the new state.
            return DriverState{
                .mode = DriverMode::DRIVE_SMOOTH,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = clip_to(-pwm_max, +pwm_max, pending_state.active_pwm),
                .lead_angle = driver_state.lead_angle,
            };
            
        case DriverMode::DRIVE_TORQUE:
            return DriverState{
                .mode = DriverMode::DRIVE_TORQUE,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .target = clip_to(-max_drive_current, +max_drive_current, pending_state.target),
            };

        case DriverMode::DRIVE_BATTERY_POWER:
            return DriverState{
                .mode = DriverMode::DRIVE_BATTERY_POWER,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .target = clip_to(-max_drive_power, +max_drive_power, pending_state.target),
            };

        case DriverMode::DRIVE_SPEED:
            return DriverState{
                .mode = DriverMode::DRIVE_SPEED,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .target = clip_to(-max_angular_speed, +max_angular_speed, pending_state.target),
            };

        case DriverMode::SEEK_ANGLE_POWER:
            return DriverState{
                .mode = DriverMode::SEEK_ANGLE_POWER,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .seek_angle = SeekAngle{
                    .target_angle = pending_state.seek_angle.target_angle,
                    .target_rotation = pending_state.seek_angle.target_rotation,
                    .max_target = clip_to(0, +max_drive_power, pending_state.seek_angle.max_target),
                    .error_integral = driver_state.seek_angle.error_integral
                }
            };

        case DriverMode::SEEK_ANGLE_TORQUE:
            return DriverState{
                .mode = DriverMode::SEEK_ANGLE_TORQUE,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .seek_angle = SeekAngle{
                    .target_angle = pending_state.seek_angle.target_angle,
                    .target_rotation = pending_state.seek_angle.target_rotation,
                    .max_target = clip_to(0, +max_drive_current, pending_state.seek_angle.max_target),
                    .error_integral = driver_state.seek_angle.error_integral
                }
            };

        case DriverMode::SEEK_ANGLE_SPEED:
            return DriverState{
                .mode = DriverMode::SEEK_ANGLE_SPEED,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .active_angle = driver_state.active_pwm != 0 ? driver_state.active_angle : readout.angle,
                .active_pwm = driver_state.active_pwm,
                .lead_angle = driver_state.lead_angle,
                .seek_angle = SeekAngle{
                    .target_angle = pending_state.seek_angle.target_angle,
                    .target_rotation = pending_state.seek_angle.target_rotation,
                    .max_target = clip_to(0, +max_angular_speed, pending_state.seek_angle.max_target),
                    .error_integral = driver_state.seek_angle.error_integral
                }
            };

        case DriverMode::RESISTANCE_CALIBRATION:
            return DriverState{
                .mode = DriverMode::RESISTANCE_CALIBRATION,
                .duration = hex_mini_drive::HISTORY_SIZE,
                .target = clip_to(0.0f, pwm_max, pending_state.target),
            };

        case DriverMode::INDUCTANCE_CALIBRATION:
            return DriverState{
                .mode = DriverMode::INDUCTANCE_CALIBRATION,
                .duration = hex_mini_drive::HISTORY_SIZE,
                .target = clip_to(0.0f, pwm_max, pending_state.target),
            };

        case DriverMode::POSITION_CALIBRATION_CHIRP:
            return DriverState{
                .mode = DriverMode::POSITION_CALIBRATION_CHIRP,
                .duration = hex_mini_drive::HISTORY_SIZE,
                .target = clip_to(-pwm_max, pwm_max, pending_state.target),
                .test_parameters = TestParameters{
                    .test_speed = clip_to(-max_angular_speed, max_angular_speed, pending_state.test_parameters.test_speed),
                    .test_duration = pending_state.test_parameters.test_duration,

                },
            };

        case DriverMode::POSITION_CALIBRATION_EMF:
            return DriverState{
                .mode = DriverMode::POSITION_CALIBRATION_EMF,
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, pending_state.duration)),
                .target = clip_to(-pwm_max, pwm_max, pending_state.target),
                .test_parameters = TestParameters{
                    .test_speed = clip_to(-max_angular_speed, max_angular_speed, pending_state.test_parameters.test_speed),
                    .test_duration = pending_state.test_parameters.test_duration,
                },
            };
    }

    return breaking_driver_state;
}

// Update the motor outputs based on the active driver state and measured phase currents and other derived values.
static inline void update_motor_control(
    DriverState & driver_state,
    hex_mini_drive::FullReadout const& readout
){
    // Update based on the active mode.
    switch (driver_state.mode) {
        
        case DriverMode::OFF:
            // Continously reset the motor outputs to breaking state.
            driver_state.motor_outputs = breaking_motor_outputs;
            return;

        case DriverMode::FREEWHEEL:
            // Continuously reset the motor outputs to freewheel state.
            driver_state.motor_outputs = freewheel_motor_outputs;
            return;

        case DriverMode::CONTINUE:
            // Continue is not a valid driver mode, only used for the pending state.
            return set_breaking_control(driver_state);

        case DriverMode::HOLD:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            // The motor outputs are already set in the setup_driver_state function; do nothing else.
            return;

        case DriverMode::SCHEDULE: 
            // We're done at the end of the schedule.
            if (
                driver_state.schedule.pointer == nullptr or 
                driver_state.schedule.current_stage >= schedule_size
            ) {
                return set_breaking_control(driver_state);
            }

            driver_state.motor_outputs = update_motor_schedule(driver_state, readout);
            return;

        case DriverMode::DRIVE_6_SECTOR:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update motor outputs for the 6 sector driving.
            driver_state.motor_outputs = update_motor_6_sector(driver_state, readout);
            return;

        case DriverMode::DRIVE_PERIODIC:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            driver_state.motor_outputs = update_motor_periodic(driver_state, readout);
            return;
                

        case DriverMode::DRIVE_SMOOTH:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the smooth driving.
            driver_state.motor_outputs = update_motor_smooth(driver_state, readout);
            return;


        case DriverMode::DRIVE_TORQUE:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the torque driving.
            driver_state.motor_outputs = update_motor_torque(driver_state, readout);
            return;

        case DriverMode::DRIVE_BATTERY_POWER:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            
            driver_state.motor_outputs = update_motor_battery_power(driver_state, readout);
            return;

        case DriverMode::DRIVE_SPEED:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            
            driver_state.motor_outputs = update_motor_speed(driver_state, readout);
            return;

        case DriverMode::SEEK_ANGLE_POWER:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the seek angle driving using power control.
            driver_state.motor_outputs = update_motor_seek_angle_power(driver_state, readout);
            return;

        case DriverMode::SEEK_ANGLE_TORQUE:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the seek angle driving using torque control.
            driver_state.motor_outputs = update_motor_seek_angle_torque(driver_state, readout);
            return;
        
        case DriverMode::SEEK_ANGLE_SPEED:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);
            
            // Update the motor outputs for the seek angle driving using speed control.
            driver_state.motor_outputs = update_motor_seek_angle_speed(driver_state, readout);
            return;

        case DriverMode::RESISTANCE_CALIBRATION:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the resistance calibration.
            driver_state.motor_outputs = update_motor_resistance_calibration(driver_state, readout);
            return;

        case DriverMode::INDUCTANCE_CALIBRATION:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the inductance calibration.
            driver_state.motor_outputs = update_motor_inductance_calibration(driver_state, readout);
            return;

        case DriverMode::POSITION_CALIBRATION_CHIRP:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the position calibration.
            driver_state.motor_outputs = update_motor_position_calibration_chirp(driver_state, readout);
            return;

        case DriverMode::POSITION_CALIBRATION_EMF:
            if (driver_state.duration-- <= 0) return set_breaking_control(driver_state);

            // Update the motor outputs for the position calibration.
            driver_state.motor_outputs = update_motor_position_calibration_emf(driver_state, readout);
            return;
    }

    // If we get here, we have an unknown/corrupted driver state.
    return set_breaking_control(driver_state);
}


// ADC readings and calculation loop
// ---------------------------------

// Process ADC readings for phase currents when the injected conversion is done.
void adc_interrupt_handler(){
    // Note: a single float assignment will cost us 5% of the CPU time (on STM32F103C8T6). We can't use floats...

    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    // Increment the readout number.
    const uint16_t readout_number = readout.readout_number + 1;

    // Start by reading the ADC conversion data
    // ----------------------------------------

    // Double check the ADC end of conversion flag was set for both ADCs.
    if (not (LL_ADC_IsActiveFlag_JEOS(ADC1) and LL_ADC_IsActiveFlag_JEOS(ADC2))) {
        // If we only get one set of readings it may be a startup timing issue, ignore it and
        // clear both flags to reset the ADCs.
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC2);
        return;
    }


    const ADCReadings adc_readings = read_adc_values();

    // Get hall sensor state
    // ---------------------

    // Read new data from the hall sensors.
    const uint8_t hall_state = read_hall_sensors_state();


    // Do the data calculations
    // ------------------------

    // Average the temperature readings since we are sampling quicker than the manufacturer indicates.
    // We can't extend the sampling time longer than it is set at the moment (about half the recommendation),
    // so we have to massage the readings for noise. Temperature varies slowly anyway.
    const float temperature = (adc_readings.temp_readout + readout.temperature * 3) * 0.25f;

    // Average out the VCC voltage; it should be relatively stable so we average to reduce our error.
    const float vcc_voltage = (adc_readings.vcc_readout * voltage_conversion + readout.vcc_voltage * 3) * 0.25f;

    // Get the motor duties that were set at the mid point of the PWM cycle, between current readings.
    const ThreePhase motor_outputs = {
        driver_state.motor_outputs.u_duty,
        driver_state.motor_outputs.v_duty,
        driver_state.motor_outputs.w_duty
    };

    const ThreePhase half_cycle_drive_voltage = adjust_to_sum_zero(motor_outputs) * (vcc_voltage * pwm_base_inverse);

    // Calculate our outputs on the motor phases. The outputs kick in halfway through the PWM cycle,
    // so we average the previous and current outputs to get the effective output for this cycle.
    // 
    // Calculate the driven phase voltages from our PWM settings and the VCC voltage. We adjust our voltages
    // such that the 0 point corresponds to the voltage at the connection point of the three phases. The
    // motor stator coils are usually connected together by the manufacturer for a star configuration motor.
    const ThreePhase drive_voltages = (previous_half_cycle_drive_voltages + half_cycle_drive_voltage) * 0.5f;

    // Store the active motor outputs for the next cycle.
    previous_half_cycle_drive_voltages = half_cycle_drive_voltage;


    // Calculate calibrated currents.
    // 
    // We need to flip the sign of the current readings. Our convention is to have settle on positive
    // current when we apply a positive PWM duty cycle to each respective phase.
    // 
    // Note that the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.
    const ThreePhase currents = adjust_to_sum_zero(ThreePhase{
        -(static_cast<float>(adc_readings.u_readout) - static_cast<float>(adc_readings.ref_readout)) * adc_to_current_units,
        -(static_cast<float>(adc_readings.v_readout) - static_cast<float>(adc_readings.ref_readout)) * adc_to_current_units,
        -(static_cast<float>(adc_readings.w_readout) - static_cast<float>(adc_readings.ref_readout)) * adc_to_current_units
    });

    // TODO: the below is no longer accurate, let's investigate later, for now keep the more basic calculation.
    // Get calibrated current divergence (the time unit is defined as 1 per cycle). We average out the
    // current diffs exactly as we do with the motor outputs; this seems to work best, can't explain why.
    //
    // I've attempted to use the finite differences approach `diff(x) = ((x[n] - x[n-1]) + (x[n+1] - x[n])) / (2 * dt)`
    // but it doesn't work as well as averaging both motor outputs and single step current diff.
    const ThreePhase currents_diff = currents - get_currents(readout);

    // Calculate the voltage drop across the coil inductance.
    const ThreePhase inductor_voltages = currents_diff * (current_calibration.inductance * current_diff_to_voltage_units);

    // Calculate the resistive voltage drop across the coil and MOSFET resistance.
    const ThreePhase resistive_voltages = currents * get_phase_resistances(current_calibration) * current_to_voltage_units;


    // Infer the back EMF voltages for each phase.
    // 
    // Calculate the EMF voltage as the remainder after subtracting the electric circuit voltages.
    // By Kirchoffs laws the total voltage of all of our components must sum to 0.
    const ThreePhase emf_voltages = inductor_voltages + resistive_voltages - drive_voltages;


    // Position Update
    // ---------------

    // Add the external offset directly to the readout angle before the update. The update will
    // calculate all values in the new angle frame; this update is ignored by the speed calculation.
    readout.angle += external_angle_offset;
    external_angle_offset = 0;

    // Predict the position; keeping track of fractional angles at the same resolution as
    // the speed. By our definition the time unit is 1 per cycle; so the angle spanned by 
    // the rotor is exactly the angular speed.
    const int32_t predicted_angle = readout.angle + static_cast<int32_t>(readout.angular_speed);


    // Switching to DQ0 Frame
    // ----------------------
    // 
    // Calculate the park transformed currents and voltages
    // 
    // Use gradient descent to estimate the inductor current angle. We don't have compute to
    // calculate the angle with atan2, so we treat it as an optimization problem over multiple
    // cycles. With cycles at 23KHz, we converge quickly, especially at high current values.

    
    // First alias the trig functions based on the predicted rotor angle.

    // Cosines of the predicted angle with respect to each phase.
    const ThreePhase three_phase_cos = get_three_phase_cos(predicted_angle);

    // Sines of the predicted angle with respect to each phase.
    const ThreePhase three_phase_sin = get_three_phase_sin(predicted_angle);
    
    // Park transform the currents and voltages: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
    // 
    // We assume the currents and emf voltages sum to 0 (eeeeh, they're close usually, works better if not adjusted).
    // 
    // In that case we can rotate our frame of reference to align ourselves with the rotor magnetic field. We then 
    // measure the current and EMF voltage projected on this line (direct) or perpendicular to it (quadrature).
    // 
    // The back EMF generated is always along the quadrature axis. The current direction is mostly under our control,
    // if we want to drive the motor efficiently we must also align the current along the quadrature axis.

    const float direct_current = dot(currents, three_phase_cos);

    const float quadrature_current = -dot(currents, three_phase_sin);

    // Invoke the CORDIC engine to compute atan2 and magnitude using the phase function.
    set_cordic(direct_current, quadrature_current);

    const float direct_emf_voltage = dot(emf_voltages, three_phase_cos);

    const float quadrature_emf_voltage = -dot(emf_voltages, three_phase_sin);


    // Current angle calculation
    // -------------------------
    // 
    // We calculate the angle of the current vector that is running through the motor coils.
    // 
    // In our convention the inductors driven with positive current form a south pole that attracts
    // the north pole of the rotor.

    // Calculate the angle at which the current is running on the motor coils. The angle offset is
    // with respect to the predicted angle as that was the angle used in the park transform.
    const auto [current_angle_offset, instant_current_magnitude] = get_cordic();

    // We can queue up the CORDIC engine for the next calculation before we read the first (I think).
    set_cordic(direct_emf_voltage, quadrature_emf_voltage);

    // Current angle in the stator frame of reference.
    const int32_t current_angle = predicted_angle + current_angle_offset;
    
    // The current measurements have a low noise floor, but it's not 0.
    const bool current_detected = instant_current_magnitude > current_measurement_minimum;
    
    // Average the current magnitude over a short duration to reduce noise.
    const float current_magnitude = (instant_current_magnitude + readout.current_magnitude * 3) * 0.25f;
    
    const float current_angular_speed = static_cast<float>(current_angle - readout.current_angle);

    // Back EMF angle observer
    // -----------------------

    // Get the angle measured from EMF relative to the predicted rotor angle.
    const auto [emf_voltage_angle_offset, instant_emf_voltage_magnitude] = get_cordic();

    // Also calculate the EMF angle for completeness.
    const int32_t instant_emf_voltage_angle = predicted_angle + emf_voltage_angle_offset;

    const int32_t predicted_emf_voltage_angle = readout.emf_voltage_angle + static_cast<int32_t>(readout.emf_voltage_angular_speed);

    const int32_t emf_angle_error = instant_emf_voltage_angle - predicted_emf_voltage_angle;

    const int32_t emf_angle_error_plus_quarter = emf_angle_error + quarter_circle;

    // Compute the angle error to the emf axis.
    // 
    // Branching if statements are easier to understand, but they do take 40 cycles extra, so we end up doing
    // it the bit bang "clever" way. Anyway, the point is we divide the angle space in 2 by and-ing the maximum
    // positive angle and then we center it on 0 by subtracting a quarter circle.
    const int32_t emf_angle_adjustment = (emf_angle_error_plus_quarter & most_positive_angle) - quarter_circle;

    // Update the new emf_voltage_angle. If the emf changes sign, we will also immediately change sign.
    // But we slowly trace to the new angle by integrating the error over time using the control parameter.
    const int32_t emf_voltage_angle = (
        predicted_emf_voltage_angle + (emf_angle_error_plus_quarter & most_negative_angle) +
        static_cast<int32_t>(emf_angle_adjustment * control_parameters.emf_angle_ki)
    );
    
    
    // Average the EMF voltage magnitude over a short duration to reduce noise.
    const float emf_voltage_magnitude = (instant_emf_voltage_magnitude + readout.emf_voltage_magnitude * 3) * 0.25f;
    
    // Measure the noise of the angle error. We can't rely on the measured error above the configured noise threshold.
    const float emf_angle_error_variance = (
        square(emf_angle_error) + 
        readout.emf_angle_error_variance * 3
    ) * 0.25f;
    
    
    // Check if the EMF angle is relatively stable. This is a proxy for detecting emf because at 0 speed, 0 emf, and
    // random noise readings for the u, v, w phases we should detect a random emf voltage of very low magnitude. Which
    // causes the angle to jump wildy and stabilize at about 90degree sqrt(variance).
    const bool emf_detected = emf_angle_error_variance < emf_angle_variance_threshold;
    
    // Use the angle variance to scale the speed adjustment, it's a simplified version of combining gaussians
    // but we use a fixed variance inverse as the normalizing factor. We're basically saying the current speed
    // is a gaussian with a variance of threshold - measured variance, the sum of the variances being fixed to
    // the emf_angle_variance_threshold. This allows us to use a precomputed inverse to avoid division.
    const float emf_variance_factor = (emf_angle_variance_threshold - emf_angle_error_variance) * emf_angle_variance_threshold_inverse;

    // Reset the emf speed to 0 if we don't have an emf detection.
    // ! Very important to use emf_detected so that the emf_variance_factor above is positive.
    const float emf_voltage_angular_speed = emf_detected * (
        readout.emf_voltage_angular_speed + 
        emf_angle_adjustment * emf_variance_factor * control_parameters.emf_angular_speed_ki
    );

    // We only get EMF when rotating, so let's get the rotation direction.
    const float emf_sign = sign(emf_voltage_angular_speed);

    // Use the rotation direction to get the absolute EMF voltage.
    const float emf_voltage_abs_angular_speed = emf_sign * emf_voltage_angular_speed;
    
    // Declare that we have an EMF reading if our speed is greater than the control parameter threshold.
    // 
    // Note that the inverse of the minimum speed is the duration it takes to rotate at said minimum speed.
    // Therefore we can only move so far during the period of time. The minimum speed also governs how
    // how often we need to probe for the real angle while stationary.
    const bool emf_fix = emf_voltage_abs_angular_speed > control_parameters.min_emf_speed;

    // Get the target angle from our EMF observer.
    const int32_t angle_from_emf = emf_voltage_angle + static_cast<int32_t>(emf_sign) * quarter_circle;

    // Track how many times we think our rotor angle is correct. Note that we keep the angle fix whilst the motor is off.
    correct_angle_counter = clip_to(
        0, angle_fix_max_count, 
        // Subtract 1 for incorrect angles; otherwise add 1 for emf or hall angle fixes.
        // Our angle is incorrect if we don't have an EMF reading whilst driving the motor.
        correct_angle_counter + ((driver_state.active_pwm and not emf_fix) ? -1 : emf_fix)
    );

    // Integreate the EMF position error only if we're detecting EMF.
    const int32_t prediction_error = emf_fix * (angle_from_emf - predicted_angle);

    // Angle update
    // ------------
    
    // Declare the angle to be correct after a threshold certainty.
    const bool angle_fix = correct_angle_counter >= angle_fix_threshold_count;
    
    const int32_t angle_adjustment = prediction_error * control_parameters.rotor_angle_ki;

    // Calculate the new angle based on the angle adjustment.
    const int32_t angle = predicted_angle + angle_adjustment;

    // Get the total angle change for the current cycle including adjustment and speed.
    const int32_t angle_diff = angle - readout.angle;
    
    // Check if the angle overflowed and count rotations. Note, we need to flag the compiler to treat
    // integer overflow as well defined behaviour!
    const int32_t rotations_increment = (
        angle_diff > 0 ? (angle < readout.angle ? +1 : 0) :
        angle_diff < 0 ? (angle > readout.angle ? -1 : 0) :
        0
    );

    // Calculate the new rotation index.
    const int32_t rotations = readout.rotations + rotations_increment;

    // Calculate speed and acceleration
    // --------------------------------

    // Calculate the rotor speed as a low pass of the detected emf speed.
    const float angular_speed_error = emf_voltage_angular_speed - readout.angular_speed;
    
    // Use the integral gain, but clamp to 0 on loss of emf.
    const float angular_speed = emf_detected * (
        readout.angular_speed + 
        angular_speed_error * control_parameters.rotor_angular_speed_ki
    );
    
    // Calculate the acceleration based on the speed change. We can use gradient descent to slowly
    // decrease our speed error. Equivalent to an exponential moving average, however framing it as
    // a gradient descent allows us to integrate the error into a higher resolution observer.
    const float acceleration_error = (angular_speed - readout.angular_speed) - readout.rotor_acceleration;

    // Update the acceleration observer.
    const float rotor_acceleration = acceleration_error * control_parameters.rotor_acceleration_ki;

    // Calculate the power use
    // -----------------------

    // Resistive power is the power dissipated in the motor coils and MOSFETs.
    const float resistive_power = dot(currents, resistive_voltages) * voltage_mul_current_to_power;

    // Inductive power is the power transfered to the motor inductors.
    const float inductive_power = dot(currents, inductor_voltages) * voltage_mul_current_to_power;

    // EMF power is the power transferred into the rotor movement, driving the motor.
    // 
    // Use the DQ0 transformed values to calculate the EMF power quickly. We also have a chance to 
    // smooth out the values to better approximate the real power use.
    const float emf_power = - (
        sign(quadrature_current) * current_magnitude * 
        sign(quadrature_emf_voltage) * emf_voltage_magnitude 
    ) * dq0_voltage_mul_current_to_power;

    // The total power is the power used from the battery. It will be positive when driving
    // the motor, meaning that we drain the battery. If this is negative it means we are charging
    // the battery by slowing down the motor (regenerative breaking).
    // 
    // The balance of all powers must be zero assuming no other source or sink of power. Thus
    // we can compute the total power from the others; mostly determined by EMF. The resistive
    // power is quite reliable and inductive_power is very small.
    const float total_power = resistive_power + inductive_power + emf_power;


    // Write the latest readout data
    // -----------------------------
    // 
    // Must update the whole state before motor pwm calculation!

    readout.readout_number = readout_number;
    
    readout.state_flags = (
        (emf_fix << emf_fix_bit_offset) |
        (emf_detected << emf_detected_bit_offset) |
        (current_detected << current_detected_bit_offset) |
        (angle_fix << angle_fix_bit_offset) |
        (hall_state << hall_state_bit_offset)
    );

    readout.u_drive_voltage = std::get<0>(drive_voltages);
    readout.v_drive_voltage = std::get<1>(drive_voltages);
    readout.w_drive_voltage = std::get<2>(drive_voltages);
    
    readout.u_current = std::get<0>(currents);
    readout.v_current = std::get<1>(currents);
    readout.w_current = std::get<2>(currents);

    readout.ref_readout = adc_readings.ref_readout;
    
    readout.u_current_diff = std::get<0>(currents_diff);
    readout.v_current_diff = std::get<1>(currents_diff);
    readout.w_current_diff = std::get<2>(currents_diff);

    readout.angle = angle;

    readout.angle_adjustment = angle_adjustment;
    readout.angular_speed = angular_speed;
    readout.vcc_voltage = vcc_voltage;

    readout.temperature = temperature;

    readout.direct_current = direct_current;
    readout.quadrature_current = quadrature_current;
    readout.direct_emf_voltage = direct_emf_voltage;
    readout.quadrature_emf_voltage = quadrature_emf_voltage;
    
    readout.total_power = total_power;
    readout.resistive_power = resistive_power;
    readout.emf_power = emf_power;
    readout.inductive_power = inductive_power;
    
    readout.emf_voltage_angle = emf_voltage_angle;
    readout.emf_voltage_magnitude = emf_voltage_magnitude;
    readout.emf_voltage_angular_speed = emf_voltage_angular_speed;
    readout.current_angle = current_angle;
    readout.current_magnitude = current_magnitude;
    readout.current_angular_speed = current_angular_speed;


    readout.rotor_acceleration = rotor_acceleration;
    readout.rotations = rotations;

    readout.emf_angle_error_variance = emf_angle_error_variance;
    
    readout.lead_angle = driver_state.lead_angle;
    readout.active_pwm = driver_state.active_pwm;
    readout.target = driver_state.target;
    readout.seek_integral = driver_state.seek_angle.error_integral;

    readout.u_resistance = current_calibration.u_resistance;
    readout.v_resistance = current_calibration.v_resistance;
    readout.w_resistance = current_calibration.w_resistance;
    readout.inductance = current_calibration.inductance;
    readout.magnetization_angle = current_calibration.magnetization_angle;
    readout.magnetization_factor = current_calibration.magnetization_factor;

    // Calculate and set motor outputs!!
    // ---------------------------------

    // Update the motor controls using the readout data.
    update_motor_control(driver_state, readout);

    // Disable the update for the control registers so we can write all 3.
    LL_TIM_DisableUpdateEvent(TIM1);

    // Send the command to the timer compare registers. Set the registers close to when cycle_end_tick 
    // is set so we can properly track the value for the next cycle. There's a half cycle delay if
    // we set the output registers too late in the cycle.
    set_motor_outputs(driver_state.motor_outputs);

    // Re-enable the update for the control registers now that we've written all 3.
    LL_TIM_EnableUpdateEvent(TIM1);


    // End of cycle
    // ------------

    // Get the tick after we've written the motor control, we need to make sure this one is 
    // within the half cycle before the pwm registers update.
    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    // Write to the latest readout if the main loop has unlocked it.
    if (not shared_readout_lock) {
        shared_readout = readout;
        shared_readout_lock = true;
    }

    // Write the latest readout to the history buffer for the main loop to read.
    readout_history_push(readout);
    if (readout_history_reset_flag) readout_history_reset();

    // Setup the new state if we were commanded by the main loop so we are prepared for the next cycle.
    // 
    // There are a few checks when copying to state, so to keep things glitchlessly fast we update it
    // after we've set the motor outputs. The new state will be ready for the next cycle.
    // 
    // Note, the new pending state may clear the readout history so this must be done after the history push.
    if (new_pending_state) {
        driver_state = setup_driver_state(driver_state, pending_state, readout);
        new_pending_state = false;
    }

    // Clear the ADC end of conversion flag so we're ready for the next conversion.
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
}