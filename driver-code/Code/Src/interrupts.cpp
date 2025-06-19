#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_pid.hpp"

#include "io.hpp"
#include "constants.hpp"
#include "error_handler.hpp"
#include "type_definitions.hpp"

#include "integer_math.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_gpio.h>


// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.

// Electrical and position state
FullReadout readout = {};
FullReadout get_readout(){
    return readout;
}

// Data queue
Readout readout_history[history_size] = {};
size_t readout_history_write_index = 0;
size_t readout_history_read_index = 0;

void readout_history_reset() {
    readout_history_write_index = 0;
    readout_history_read_index = 0;
}
bool readout_history_full(){
    return readout_history_write_index >= history_size;
}
bool readout_history_available(){
    return readout_history_read_index < readout_history_write_index;
}
Readout readout_history_pop(){
    Readout readout = readout_history[readout_history_read_index];
    readout_history_read_index += 1;
    // Reset both indexes if we have sent the whole history.
    if (readout_history_read_index >= history_size) readout_history_reset();
    return readout;
}

static inline bool readout_history_push(Readout const & readout){
    if (readout_history_write_index >= history_size) return false;
    readout_history[readout_history_write_index] = readout;
    readout_history_write_index += 1;
    return true;
}

// State Data
// ----------


// Currently active motor outputs.
MotorOutputs active_motor_outputs = null_motor_outputs;

// We now set the motor outputs one turn delayed :( because our update takes more than half a PWM cycle.
MotorOutputs previous_motor_outputs = null_motor_outputs;

PIDControl current_angle_control = {};
PIDControl torque_control = {};


// Hall sensors
// ------------

// Hall states as bits, 0b001 = hall 1, 0b010 = hall 2, 0b100 = hall 3.
uint8_t hall_state = 0b000;

// The 6 valid configurations of hall sensors in trigonometric order.
// 
// The order is u, uv, v, vw, w, wu. Treat values outside the range as
// invalid, the magnet is not present or the sensors are not ready.
uint8_t hall_sector = hall_sector_base;

// Whether the position is valid.
bool angle_valid = false;

// Best estimate of the **electric** angle and speed (that is the angle between the coil 
// U and the magnetic North). The actual position of the output depends on the number of
// pole pairs, and the slot triplets, and the mechanical gear ratio.
PositionStatistics electric_position = null_position_statistics;

// Combine the motor duty cycles into a single 32-bit value; because it barely fits.
static inline uint32_t encode_pwm_commands(MotorOutputs const & outputs){
    return (
        outputs.u_duty * pwm_base * pwm_base +
        outputs.v_duty * pwm_base +
        outputs.w_duty
    );
}

static inline MotorOutputs mid_motor_outputs(MotorOutputs const & a, MotorOutputs const & b){
    return MotorOutputs{
        .u_duty = static_cast<uint16_t>((a.u_duty + b.u_duty) / 2),
        .v_duty = static_cast<uint16_t>((a.v_duty + b.v_duty) / 2),
        .w_duty = static_cast<uint16_t>((a.w_duty + b.w_duty) / 2)
    };
}




// Motor driver state
// ------------------

DriverState driver_state = DriverState::OFF;

uint16_t pwm_target = 0;

void set_pwm_target(uint16_t pwm){
    // Set the PWM target for the motor drive.
    pwm_target = clip_to(0, pwm_max, pwm);
}

int16_t leading_angle = 0;

void set_leading_angle(int16_t angle){
    // Set the leading angle for the motor drive.
    leading_angle = clip_to(0, angle_base-1, angle);
}


PWMSchedule const* schedule_queued = nullptr;

// Active test schedule.
PWMSchedule const* schedule_active = nullptr;
size_t schedule_counter = 0;
size_t schedule_stage = 0;

static inline PWMSchedule const* get_and_reset_schedule_queued(){
    PWMSchedule const* schedule = schedule_queued;
    schedule_queued = nullptr;
    return schedule;
}

// Motor Control
// -------------

const int min_sweep_offset = - 45 * angle_base / 360;
const int max_sweep_offset = + 45 * angle_base / 360;
const int sweep_step = 1 * angle_base / 360;

int sweep_offset = min_sweep_offset;
int sweep_direction = +1;

bool keep_pid_controls = false;

static inline void reset_pid_controls(){
    // Reset the PID controls to their initial state.
    current_angle_control = PIDControl{};
    torque_control = PIDControl{};
    sweep_offset = min_sweep_offset;
    sweep_direction = +1;
}


bool is_motor_stopped(){
    // Check if the motor is stopped.
    return (driver_state == DriverState::OFF) || (driver_state == DriverState::FREEWHEEL);
}

void set_motor_state(MotorOutputs const & outputs, const DriverState state){
    switch(state){
        // Short all motor phases to ground quickly stopping the motor.
        case DriverState::OFF:
            active_motor_outputs = set_motor_outputs(null_motor_outputs);
            enable_motor_outputs();
            driver_state = DriverState::OFF;
            return;
        case DriverState::FREEWHEEL:
            // Disable the motor outputs; leaving them floating voltage/tristate.
            disable_motor_outputs();
            active_motor_outputs = set_motor_outputs(null_motor_outputs);
            driver_state = DriverState::FREEWHEEL;
            return;
        case DriverState::HOLD:
            // Set the motor outputs to hold the current position.
            active_motor_outputs = set_motor_outputs(MotorOutputs{
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, outputs.duration)),
                .u_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, outputs.u_duty)),
                .v_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, outputs.v_duty)),
                .w_duty = static_cast<uint16_t>(clip_to(0, pwm_max_hold, outputs.w_duty))
            });
            enable_motor_outputs();
            driver_state = DriverState::HOLD;
            return;
        case DriverState::DRIVE_POS:
        case DriverState::DRIVE_NEG:
            // Set the motor outputs to drive the motor in the positive or negative direction.
            active_motor_outputs = set_motor_outputs(MotorOutputs{
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, outputs.duration)),
                .u_duty = 0,
                .v_duty = 0,
                .w_duty = 0
            });
            enable_motor_outputs();
            driver_state = state;
            return;
        case DriverState::DRIVE_SMOOTH_POS:
        case DriverState::DRIVE_SMOOTH_NEG:
            // Set the motor outputs to drive the motor in the smooth position mode.
            active_motor_outputs = set_motor_outputs(MotorOutputs{
                .duration = static_cast<uint16_t>(clip_to(0, max_timeout, outputs.duration)),
                .u_duty = 0,
                .v_duty = 0,
                .w_duty = 0
            });
            enable_motor_outputs();
            driver_state = state;
            return;
        case DriverState::SCHEDULE:
            schedule_queued = reinterpret_cast<PWMSchedule const*>(&outputs);
            driver_state = DriverState::SCHEDULE;
            return;
    }

    // We shouldn't reach here, enable compiler warnings!
    return error();
}



static inline void update_motor_hold(){
    // Set the duty cycle and hold.
    active_motor_outputs = set_motor_outputs(active_motor_outputs);
    enable_motor_outputs();

    if (not active_motor_outputs.duration) return set_motor_break();
}

static inline void update_motor_sector(const uint8_t hall_sector, const uint16_t (* motor_sector_driving_table)[3]){
    if (motor_sector_driving_table == nullptr) return error();
    
    // Use the hall sensor state to determine the motor position and commutation.
    if (hall_sector >= hall_sector_base) return set_motor_break();


    // Get the voltage for the three phases from the table.

    const uint16_t voltage_phase_u = motor_sector_driving_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_sector_driving_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_sector_driving_table[hall_sector][2];

    active_motor_outputs = set_motor_outputs(MotorOutputs{
        .duration = active_motor_outputs.duration,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm_target / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm_target / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm_target / pwm_base)
    });

    enable_motor_outputs();

    if (not active_motor_outputs.duration) return set_motor_break();
}

static inline void update_motor_smooth(
    const int direction, 
    const bool angle_valid, 
    const int angle, 
    const int angular_speed,
    const int current_angle_offset,
    const int torque_current
){
    if (not angle_valid) return set_motor_break();

    const bool is_motor_moving = abs(angular_speed) > threshold_speed;

    const int target_lead_angle = direction * (is_motor_moving ? 
        leading_angle : 
        leading_angle + sweep_offset
    );

    if (is_motor_moving) {
        sweep_offset = min_sweep_offset;
        sweep_direction = +1;
    } else {
        sweep_offset += sweep_step * sweep_direction;
        if (sweep_offset > max_sweep_offset) {
            sweep_offset = max_sweep_offset;
            sweep_direction = -1;
        }
        if (sweep_offset < min_sweep_offset) {
            sweep_offset = min_sweep_offset;
            sweep_direction = +1;
        }

    }

    const int current_angle_relative = signed_angle(current_angle_offset - target_lead_angle);
    
    current_angle_control = compute_pid_control(
        pid_parameters.current_angle_gains,
        current_angle_control,
        current_angle_relative,
        0
    );

    const int current_target = max_drive_current * pwm_target / pwm_base + (is_motor_moving ? 0 : min_drive_current);

    torque_control = compute_pid_control(
        pid_parameters.torque_gains,
        torque_control,
        max(0, direction * torque_current),
        current_target
    );

    const int pwm = clip_to(0, pwm_max, torque_control.output);

    const int target_angle = normalize_angle(angle + target_lead_angle + current_angle_control.output);

    const uint16_t voltage_phase_u = get_phase_pwm(target_angle);
    const uint16_t voltage_phase_v = get_phase_pwm(target_angle - third_circle);
    const uint16_t voltage_phase_w = get_phase_pwm(target_angle - two_thirds_circle);

    active_motor_outputs = set_motor_outputs(MotorOutputs{
        .duration = active_motor_outputs.duration,
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm / pwm_base),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm / pwm_base),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm / pwm_base)
    });

    enable_motor_outputs();

    if (not active_motor_outputs.duration) return set_motor_break();
}

// Quickly update the PWM settings from the test schedule.
static inline void update_motor_schedule(){
    // Start a new schedule if the old one is finished.
    if(schedule_active == nullptr){
        // Get the queued schedule and reset the variable for a new command.
        schedule_active = get_and_reset_schedule_queued();

        
        // We should not enter testing mode without a valid schedule.
        if (schedule_active == nullptr) return error();
        
        // Clear the readouts buffer of old data.
        readout_history_reset();
        
        // Reset the schedule counter.
        schedule_stage = 0;
        schedule_counter = 0;
    }

    // Check if we reached the end of the schedule; and stop.
    if (schedule_stage >= schedule_size) {
        schedule_active = nullptr;   
        return set_motor_break();
    }

    const PWMSchedule & schedule = *schedule_active;

    active_motor_outputs = set_motor_outputs(schedule[schedule_stage]);

    enable_motor_outputs();

    // Go to the next step in the schedule.
    schedule_counter += 1;
    if (schedule_counter >= schedule[schedule_stage].duration) {
        schedule_stage += 1;
        schedule_counter = 0;
    }
}



// Interrupt handlers
// ------------------

// These functions are called by the autogenerated C code.

// Critical function!! 23KHz PWM cycle
// -----------------------------------
// Process ADC readings for phase currents when the injected conversion is done.
void adc_interrupt_handler(){
    if (not LL_ADC_IsActiveFlag_JEOS(ADC1)) error(); 

    // Note: a single float assignment will costs us 5% of the CPU time (on STM32F103C8T6). We can't use floats...


    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    // If we managed to set the compare before the new cycle then it's the mid point; otherwise it's the previous outputs.
    const auto motor_outputs = readout.cycle_end_tick < pwm_base ? 
        previous_motor_outputs : 
        mid_motor_outputs(previous_motor_outputs, active_motor_outputs);
    previous_motor_outputs = active_motor_outputs;

    
    // We read the current at the halfway point of the PWM cycle. It is
    // most accurate to use the average PWM of the last 2 duty cycles. 
    readout.pwm_commands = encode_pwm_commands(motor_outputs);


    // Write the current readout index.
    readout.readout_number = readout.readout_number + 1;

    // U and W phases are measured at the same time, followed by V and the reference voltage.
    // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
    // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
    // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
    
    // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
    // The PWM period lasts 1/23.4KHz = 42.7us.

    const uint16_t ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3);
    readout.ref_readout = ref_readout;

    // I wired the shunt resistors in the wrong way, so we need to flip the sign of the current readings.
    const int u_readout = -(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - ref_readout);
    // Flip the sign of V because we accidentally wired it the other way (the right way...). Oopsie doopsie.
    const int v_readout = +(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - ref_readout);
    const int w_readout = -(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - ref_readout);

    const int u_readout_diff = u_readout - readout.u_readout;
    const int v_readout_diff = v_readout - readout.v_readout;
    const int w_readout_diff = w_readout - readout.w_readout;

    readout.u_readout = u_readout;
    readout.v_readout = v_readout;
    readout.w_readout = w_readout;

    readout.u_readout_diff = u_readout_diff;
    readout.v_readout_diff = v_readout_diff;
    readout.w_readout_diff = w_readout_diff;


    const uint16_t instant_temperature = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    const uint16_t instant_vcc_voltage = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);

    readout.instant_vcc_voltage = instant_vcc_voltage;

    // Note the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.
    readout.temperature = round_div(instant_temperature * 1 + readout.temperature * 15, 16);

    const int vcc_voltage = round_div(readout.instant_vcc_voltage * 4 + readout.vcc_voltage * 12, 16);
    readout.vcc_voltage = vcc_voltage;


        // Predict the position based on the previous state.
    const auto predicted_position = predict_position(electric_position);

    // Read new data from the hall sensors.
    hall_state = read_hall_sensors_state();

    // Remember the previous hall sector to compute transitions.
    const uint8_t previous_hall_sector = hall_sector;

    // Update the sector variable.
    hall_sector = get_hall_sector(hall_state);

    // Check if the magnet is present.
    angle_valid = hall_sector < hall_sector_base;

    // We need 2 hall sector readings in a row to compute an update.
    const bool previous_angle_valid = previous_hall_sector < hall_sector_base;

    // Update the position; or reset to default.
    electric_position = (angle_valid ? previous_angle_valid ? 
        // Perform the Kalman filter update based on the hall sensor data.
        infer_position_from_hall_sensors(
            predicted_position,
            hall_sector,
            previous_hall_sector
        ) : 
        // If the previous sector was invalid; initialize the position.
        get_default_sector_position(hall_sector) : 
        // No valid sector; reset the position.
        null_position_statistics
    );


    readout.position = (electric_position.angle & 0x3FF) | (hall_state << 13) | angle_valid << 12;
    readout.angle_variance = electric_position.angle_variance;
    readout.angular_speed = electric_position.angular_speed;
    readout.angular_speed_variance = electric_position.angular_speed_variance;


    const int scaled_u_current = round_div(u_readout * current_calibration.u_factor, current_calibration_fixed_point);
    const int scaled_v_current = round_div(v_readout * current_calibration.v_factor, current_calibration_fixed_point);
    const int scaled_w_current = round_div(w_readout * current_calibration.w_factor, current_calibration_fixed_point);

    // The current sum should be zero, but we can have an offset due to (uncompensated) differences in the shunt resistors.
    const int avg_current = round_div(scaled_u_current + scaled_v_current + scaled_w_current, 3);

    const int u_current = scaled_u_current - avg_current;
    const int v_current = scaled_v_current - avg_current;
    const int w_current = scaled_w_current - avg_current;


    const int scaled_u_current_diff = signed_round_div(u_readout_diff * current_calibration.u_factor, current_calibration_fixed_point);
    const int scaled_v_current_diff = signed_round_div(v_readout_diff * current_calibration.v_factor, current_calibration_fixed_point);
    const int scaled_w_current_diff = signed_round_div(w_readout_diff * current_calibration.w_factor, current_calibration_fixed_point);
    const int avg_current_diff = signed_round_div(scaled_u_current_diff + scaled_v_current_diff + scaled_w_current_diff, 3);

    const int u_current_diff = scaled_u_current_diff - avg_current_diff;
    const int v_current_diff = scaled_v_current_diff - avg_current_diff;
    const int w_current_diff = scaled_w_current_diff - avg_current_diff;

    const int diff_to_voltage = round_div(phase_readout_diff_per_cycle_to_voltage * current_calibration.inductance_factor, current_calibration_fixed_point);

    const int u_inductor_voltage = signed_round_div(u_current_diff * diff_to_voltage, current_fixed_point);
    const int v_inductor_voltage = signed_round_div(v_current_diff * diff_to_voltage, current_fixed_point);
    const int w_inductor_voltage = signed_round_div(w_current_diff * diff_to_voltage, current_fixed_point);

    const int u_resistive_voltage = round_div(
        round_div(u_current * phase_resistance, resistance_fixed_point) * voltage_fixed_point, 
        current_fixed_point
    );
    const int v_resistive_voltage = round_div(
        round_div(v_current * phase_resistance, resistance_fixed_point) * voltage_fixed_point, 
        current_fixed_point
    );
    const int w_resistive_voltage = round_div(
        round_div(w_current * phase_resistance, resistance_fixed_point) * voltage_fixed_point, 
        current_fixed_point
    );

    const int u_drive_voltage_scaled = round_div(motor_outputs.u_duty * vcc_voltage, pwm_base);
    const int v_drive_voltage_scaled = round_div(motor_outputs.v_duty * vcc_voltage, pwm_base);
    const int w_drive_voltage_scaled = round_div(motor_outputs.w_duty * vcc_voltage, pwm_base);
    const int avg_drive_voltage = round_div(u_drive_voltage_scaled + v_drive_voltage_scaled + w_drive_voltage_scaled, 3);

    const int u_drive_voltage = u_drive_voltage_scaled - avg_drive_voltage;
    const int v_drive_voltage = v_drive_voltage_scaled - avg_drive_voltage;
    const int w_drive_voltage = w_drive_voltage_scaled - avg_drive_voltage;

    const int u_emf_voltage = u_resistive_voltage + u_inductor_voltage - u_drive_voltage;
    const int v_emf_voltage = v_resistive_voltage + v_inductor_voltage - v_drive_voltage;
    const int w_emf_voltage = w_resistive_voltage + w_inductor_voltage - w_drive_voltage;

    // Calculate the park transformed currents (unscaled).
    //
    // Use the kalman filtered angle estimates. These are updated on the order of 2KHz while the PWM
    // cycle is at 23KHz. The angle might diverge from reality and we can sense it by the misalignment
    // in the park transformed currents.
    // 
    // The beta component contributes to torque. The alpha component should be driven to zero, but it
    // can also be used to hold the motor at standstill. When we stay within a single hall sector, the
    // angle estimate degrades to +-30 degrees. A holding current would exert a torque proportional to
    // the deviation between the real and target angle even when we can't sense the real angle.


    const int alpha_current = (
        u_current * get_cos(electric_position.angle) / angle_base +
        v_current * get_cos(electric_position.angle - third_circle) / angle_base +
        w_current * get_cos(electric_position.angle - two_thirds_circle) / angle_base
    );

    const int beta_current = (
        u_current * -get_sin(electric_position.angle) / angle_base +
        v_current * -get_sin(electric_position.angle - third_circle) / angle_base +
        w_current * -get_sin(electric_position.angle - two_thirds_circle) / angle_base
    );

    const auto [current_angle_offset, current_magnitude] = int_atan2(beta_current, alpha_current);

    readout.current_angle_offset = current_angle_offset;

    // We definitely need the code below, but can't afford to run it on the STM32F103C8T6, need faster chip!

    // const int alpha_emf_voltage = (
    //     u_emf_voltage * get_cos(electric_position.angle) / angle_base +
    //     v_emf_voltage * get_cos(electric_position.angle - third_circle) / angle_base +
    //     w_emf_voltage * get_cos(electric_position.angle - two_thirds_circle) / angle_base
    // );
    //
    // const int beta_emf_voltage = (
    //     u_emf_voltage * -get_sin(electric_position.angle) / angle_base +
    //     v_emf_voltage * -get_sin(electric_position.angle - third_circle) / angle_base +
    //     w_emf_voltage * -get_sin(electric_position.angle - two_thirds_circle) / angle_base
    // );
    //
    // const auto [emf_voltage_angle_offset, emf_magnitude] = int_atan2(beta_emf_voltage, alpha_emf_voltage);
    
    // readout.emf_voltage_angle_offset = emf_voltage_angle_offset;


    readout.total_power = -signed_round_div(
        power_fixed_point * signed_round_div(
            u_current * u_drive_voltage + 
            v_current * v_drive_voltage + 
            w_current * w_drive_voltage,
            voltage_fixed_point
        ), 
        current_fixed_point
    );

    readout.resistive_power = round_div(
        power_fixed_point * round_div(
            u_current * u_resistive_voltage + 
            v_current * v_resistive_voltage + 
            w_current * w_resistive_voltage,
            voltage_fixed_point
        ), 
        current_fixed_point
    );

    readout.emf_power = -signed_round_div(
        power_fixed_point * signed_round_div(
            u_current * u_emf_voltage + 
            v_current * v_emf_voltage + 
            w_current * w_emf_voltage,
            voltage_fixed_point
        ), 
        current_fixed_point
    );

    readout.inductive_power = signed_round_div(
        power_fixed_point * signed_round_div(
            u_current * u_inductor_voltage + 
            v_current * v_inductor_voltage + 
            w_current * w_inductor_voltage,
            voltage_fixed_point
        ),
        current_fixed_point
    );
    
    bool used_pid_controls = false;

    // Update motor control.
    switch (driver_state) {
        case DriverState::OFF:
            active_motor_outputs = set_motor_outputs(null_motor_outputs);
            enable_motor_outputs();
            break;
        case DriverState::FREEWHEEL:
            disable_motor_outputs();
            active_motor_outputs = set_motor_outputs(null_motor_outputs);
            break;
        case DriverState::SCHEDULE:
            update_motor_schedule();
            break;
        case DriverState::DRIVE_POS:
            update_motor_sector(hall_sector, motor_sector_driving_pos);
            break;
        case DriverState::DRIVE_NEG:
            update_motor_sector(hall_sector, motor_sector_driving_neg);
            break;
        case DriverState::DRIVE_SMOOTH_POS:
            update_motor_smooth(+1, angle_valid, electric_position.angle, electric_position.angular_speed, current_angle_offset, beta_current);
            used_pid_controls = true;
            break;
        case DriverState::DRIVE_SMOOTH_NEG:
            update_motor_smooth(-1, angle_valid, electric_position.angle, electric_position.angular_speed, current_angle_offset, beta_current);
            used_pid_controls = true;
            break;
        case DriverState::HOLD:
            update_motor_hold();
            break;
    }

    if (keep_pid_controls and not used_pid_controls) {
        keep_pid_controls = false;
        reset_pid_controls();
    } else if (used_pid_controls and not keep_pid_controls) {
        // If we used the PID controls, we want to keep them for the next cycle.
        keep_pid_controls = true;
    }

    readout.current_angle_error = current_angle_control.error;
    readout.current_angle_control = current_angle_control.output;
    readout.current_angle_integral = current_angle_control.integral / gains_fixed_point;
    readout.current_angle_derivative = current_angle_control.derivative / gains_fixed_point;

    readout.torque_error = torque_control.error;
    readout.torque_control = torque_control.output;
    readout.torque_integral = torque_control.integral / gains_fixed_point;
    readout.torque_derivative = torque_control.derivative / gains_fixed_point;

    // Send data to the main loop after updating the PWM registers; the queue access might be slow.
    
    // Try to write the latest readout if there's space.
    readout_history_push(readout);



    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    LL_ADC_ClearFlag_JEOS(ADC1);
}


// Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    // We shouldn't trigger this, but including for documentation.
    error();
    
    // Note, this updates on both up and down counting, get direction 
    // with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
}


void tim2_global_handler(){
    // We shouldn't trigger this, but including for documentation.
    error();

    // The TIM2 updates at a frequency of about 1KHz. Our motor might rotate slower than this
    // so we have to count updates (overflows) between hall sensor triggers.

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        // hall_observed_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);

        if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
            // We have a hall sensor toggle and an update event at the same time; clear the update flag.
            LL_TIM_ClearFlag_UPDATE(TIM2);
        }
    }
    
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        // We overflowed the timer; this means we haven't seen a hall sensor toggle in a while.
        // hall_unobserved_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);
    }
}


// Write only variable, for funsies.
volatile int write_only = 0;

void initialize_angle_tracking(){
    // Load the phase and sin tables into memory.
    for (int i = 0; i < angle_base; i++) {
        write_only = get_phase_pwm(i);
        write_only = get_sin(i);
    }
}
