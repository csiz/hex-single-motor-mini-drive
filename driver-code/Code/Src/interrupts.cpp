#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_motor.hpp"

#include "user_data.hpp"

#include "io.hpp"
#include "constants.hpp"
#include "error_handler.hpp"

#include "integer_math.hpp"

#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_gpio.h>


// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.

// Timing data
uint32_t adc_update_number = 0;
uint32_t get_adc_update_number(){
    return adc_update_number;
}
uint32_t hall_unobserved_number = 0;
uint32_t get_hall_unobserved_number(){
    return hall_unobserved_number;
}
uint32_t hall_observed_number = 0;
uint32_t get_hall_observed_number(){
    return hall_observed_number;
}

// Electrical state
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

uint16_t prev_u_pwm_duty = 0;
uint16_t prev_v_pwm_duty = 0;
uint16_t prev_w_pwm_duty = 0;

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


// Critical function!! 23KHz PWM cycle
// -----------------------------------

static inline void pwm_cycle_and_adc_update(){
    // Note: a single float assignment will costs us 5% of the CPU time (on STM32F103C8T6). We can't use floats...


    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    
    // Get the pwm duty cycle to this readout, it should have been active during the half period before the ADC sampling.
    const uint16_t active_u_pwm_duty = get_motor_u_pwm_duty();
    const uint16_t active_v_pwm_duty = get_motor_v_pwm_duty();
    const uint16_t active_w_pwm_duty = get_motor_w_pwm_duty();
    
    // We read the current at the halfway point of the PWM cycle. It is
    // most accurate to use the average PWM of the last 2 duty cycles. 
    readout.pwm_commands = (
        (prev_u_pwm_duty + active_u_pwm_duty) / 2 * pwm_base * pwm_base + 
        (prev_v_pwm_duty + active_v_pwm_duty) / 2 * pwm_base + 
        (prev_w_pwm_duty + active_w_pwm_duty) / 2
    );

    prev_u_pwm_duty = active_u_pwm_duty;
    prev_v_pwm_duty = active_v_pwm_duty;
    prev_w_pwm_duty = active_w_pwm_duty;

    // Write the current readout index.
    readout.readout_number = adc_update_number;
    
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
    readout.instant_vcc_voltage = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);

    // Note the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.
    readout.temperature = (instant_temperature * 1 + readout.temperature * 15) / 16;
    readout.vcc_voltage = (readout.instant_vcc_voltage * 4 + readout.vcc_voltage * 12) / 16;


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

    
    const int scaled_u_current = u_readout * current_calibration.u_factor / current_calibration_fixed_point;
    const int scaled_v_current = v_readout * current_calibration.v_factor / current_calibration_fixed_point;
    const int scaled_w_current = w_readout * current_calibration.w_factor / current_calibration_fixed_point;

    // The current sum should be zero, but we can have an offset due to (uncompensated) differences in the shunt resistors.
    const int avg_current = (scaled_u_current + scaled_v_current + scaled_w_current) / 3;

    const int u_current = scaled_u_current - avg_current;
    const int v_current = scaled_v_current - avg_current;
    const int w_current = scaled_w_current - avg_current;


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

    // For cos lookup we can use the sin lookup table + 90 degrees (quarter_circle).
    const int alpha_current = (
        u_current * get_sin(electric_position.angle + quarter_circle) / angle_base +
        v_current * get_sin(electric_position.angle + quarter_circle - third_circle) / angle_base +
        w_current * get_sin(electric_position.angle + quarter_circle - two_thirds_circle) / angle_base);

    const int beta_current = (
        u_current * -get_sin(electric_position.angle) / angle_base +
        v_current * -get_sin(electric_position.angle - third_circle) / angle_base +
        w_current * -get_sin(electric_position.angle - two_thirds_circle) / angle_base);


    // TODO: these 2 are not correct, figure out the proper units.
    readout.emf_power = beta_current;
    readout.inductive_power = alpha_current;
    

    const auto [current_angle_offset, current_magnitude] = int_atan2(beta_current, alpha_current);

    readout.current_angle_offset = current_angle_offset;
    readout.current_angle = normalize_angle(electric_position.angle + current_angle_offset);

    // TODO: compute current_angle_offset_variance based on the past angle estimates.

    readout.total_power = readout.vcc_voltage * (
        u_current * get_motor_u_pwm_duty() + 
        v_current * get_motor_v_pwm_duty() + 
        w_current * get_motor_w_pwm_duty()) / pwm_base;

    readout.resistive_power = power_fixed_point * phase_resistance / resistance_fixed_point * (
        u_current * u_current + 
        v_current * v_current + 
        w_current * w_current) / square(current_fixed_point);
     

    // Update motor control.
    switch (driver_state) {
        case DriverState::OFF:
            motor_break();
            break;
        case DriverState::FREEWHEEL:
            motor_freewheel();
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
            update_motor_smooth(angle_valid, electric_position.angle, +1);
            break;
        case DriverState::DRIVE_SMOOTH_NEG:
            update_motor_smooth(angle_valid, electric_position.angle, -1);
            break;
        case DriverState::HOLD:
            update_motor_hold();
            break;
    }

    // Send data to the main loop after updating the PWM registers; the queue access might be slow.
    
    // Try to write the latest readout if there's space.
    readout_history_push(readout);



    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
}

// Interrupt handlers
// ------------------

// These functions are called by the autogenerated C code.

void adc_interrupt_handler(){
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        // Process ADC readings for phase currents when the injected conversion is done.
        pwm_cycle_and_adc_update();

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        error();
    }
}


// Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    // We shouldn't trigger this, but including for documentation.
    error();
    // Note, this updates on both up and down counting, get direction 
    // with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
}


void tim2_global_handler(){
    // The TIM2 updates at a frequency of about 1KHz. Our motor might rotate slower than this
    // so we have to count updates (overflows) between hall sensor triggers.

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        hall_observed_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);

        if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
            // We have a hall sensor toggle and an update event at the same time; clear the update flag.
            LL_TIM_ClearFlag_UPDATE(TIM2);
        }
    }
    
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        // We overflowed the timer; this means we haven't seen a hall sensor toggle in a while.
        hall_unobserved_number += 1;
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
