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



// Critical function!! 23KHz PWM cycle
// -----------------------------------

static inline void pwm_cycle_and_adc_update(){
    // Note: a single float assignment will costs us 5% of the CPU time (on STM32F103C8T6). We can't use floats...


    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    increment_time_since_observation();

    // Write the previous pwm duty cycle to this readout, it should have been active during the prior to the ADC sampling.
    readout.pwm_commands = get_motor_u_pwm_duty() * pwm_base * pwm_base + get_motor_v_pwm_duty() * pwm_base + get_motor_w_pwm_duty();
    
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


    const int angle = normalize_angle(angle_at_observation + angular_speed_at_observation * time_since_observation / speed_scale);
    
    readout.position = (angle & 0x3FF) | (hall_state << 13) | angle_valid << 12;

    readout.angular_speed = angular_speed_at_observation;
    readout.angle_variance = angle_variance_at_observation;
    readout.angular_speed_variance = angular_speed_variance_at_observation;

    
    const int scaled_u_current = u_readout * current_calibration.u_factor / current_calibration_base;
    const int scaled_v_current = v_readout * current_calibration.v_factor / current_calibration_base;
    const int scaled_w_current = w_readout * current_calibration.w_factor / current_calibration_base;

    // The current sum should be zero, but we can have an offset due to (uncompensated) differences in the shunt resistors.
    const int avg_current = (scaled_u_current + scaled_v_current + scaled_w_current) / 3;

    const int u_current = scaled_u_current - avg_current;
    const int v_current = scaled_v_current - avg_current;
    const int w_current = scaled_w_current - avg_current;

    // TODO: also correct the diffs for the average offset?

    
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
        u_current * sin_lookup[normalize_angle(angle + quarter_circle)] / angle_base +
        v_current * sin_lookup[normalize_angle(angle + quarter_circle - third_circle)] / angle_base +
        w_current * sin_lookup[normalize_angle(angle + quarter_circle - two_thirds_circle)] / angle_base);

    const int beta_current = (
        u_current * -sin_lookup[angle] / angle_base +
        v_current * -sin_lookup[normalize_angle(angle - third_circle)] / angle_base +
        w_current * -sin_lookup[normalize_angle(angle - two_thirds_circle)] / angle_base);


    // TODO: these 2 are not correct, figure out the proper units.
    readout.emf_power = beta_current;
    readout.inductive_power = alpha_current;
    

    const auto [current_angle_offset, current_magnitude] = atan2_integer(beta_current, alpha_current);

    readout.current_angle_offset = current_angle_offset;
    readout.current_angle = normalize_angle(angle + current_angle_offset);

    // TODO: compute current_angle_offset_variance based on the past angle estimates.

    readout.total_power = readout.vcc_voltage * (
        u_current * get_motor_u_pwm_duty() + 
        v_current * get_motor_v_pwm_duty() + 
        w_current * get_motor_w_pwm_duty()) / pwm_base;

    readout.resistive_power = phase_int_resistance * (
        u_current * u_current + 
        v_current * v_current + 
        w_current * w_current) / 256;
     

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
            update_motor_smooth(angle_valid, angle, +1);
            break;
        case DriverState::DRIVE_SMOOTH_NEG:
            update_motor_smooth(angle_valid, angle, -1);
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
        update_position_observation();

        hall_observed_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
        if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
            // We have a hall sensor toggle and an update event at the same time; clear the update flag.
            LL_TIM_ClearFlag_UPDATE(TIM2);
        }
    } else if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        // We overflowed the timer; this means we haven't seen a hall sensor toggle in a while.
        update_position_unobserved();

        hall_unobserved_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);

    } else {
        error();
    }
}

// Initializing ADC, interrupts and timers
// ---------------------------------------

void initialize_angle_tracking(){
    update_position_observation();
}

void adc_init(){


    // Enable the ADC interrupt for the end of the injected sequence.
    LL_ADC_EnableIT_JEOS(ADC1);
    // Disable the ADC interrupt for the end of the regular sequence; we are not using it.
    LL_ADC_DisableIT_EOS(ADC1);

    // Enable the ADCs and wait for them to settle.
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    HAL_Delay(100);

    // Calibrate the ADCs.
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_StartCalibration(ADC2);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {}

    // Start ADC conversions from the external trigger in TIM1.
    LL_ADC_INJ_StartConversionExtTrig(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_StartConversionExtTrig(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    // Gotta enable ext triggers for the second channel to work in dual mode.
    LL_ADC_REG_StartConversionExtTrig(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
}


void interrupts_init(){

    // TIM1 is used to generate the PWM signals for the motor phases, and to trigger the ADC.
    // 
    // Note that we don't need to set dead-time at this point; the gate driver FD6288T&Q has a 
    // built-in dead-time of 100-300ns. The MOSFETs AO4266E have turn on rise time of 8ns and 
    // turn off fall time of 22ns so we should be covered.

    // Disable TIM1 update interrupt; occuring every PWM cycle. We will use the ADC interrupt to update the motor control.
    LL_TIM_DisableIT_UPDATE(TIM1);
    // Disable TIM1 trigger interrupt; it's only triggered during resets.
    LL_TIM_DisableIT_TRIG(TIM1);
    // Disable commutation interrupts; 
    LL_TIM_DisableIT_COM(TIM1);


    // Enable TIM2 update interrupt. We need to count updates of TIM2 to measure slow motor 
    // speed in case the timer overflows before a toggle in the hall sensor.
    LL_TIM_EnableIT_UPDATE(TIM2);
    // Disable TIM2 trigger interrupt. It is triggered every reset and hall sensor toggle.
    LL_TIM_DisableIT_TRIG(TIM2);
    // Enable TIM2 channel 1 interrupt. It is triggered every hall sensor toggle; we can 
    // read the time from the last time the input changed.
    LL_TIM_EnableIT_CC1(TIM2);
    // Disable TIM2 channel 2 interrupt. We are ignoring the delayed trigger output from 
    // TIM2 as we will commutate faster than the hall sensor toggles. To use this effectively
    // we need to use the prescaler to slow the timer down; but then we lose resolution.
    LL_TIM_DisableIT_CC2(TIM2);

    // Disable TIM1 capture/compare update selection preload. This is used to update the
    // output modes of all channels at the same time when a commutation event is triggered.
    // We don't need this as we always enable PWM mode and update the compare register to 
    // modulate the PWM duty cycle.
    // 
    // This setting is mostly here to document that in order to trigger commutations on TRGI 
    // we also need to enable preload for the motor ouput channel mode registers.
    LL_TIM_CC_DisablePreload(TIM1);
    // Don't allow TIM1 commutations to be triggered by TIM2 hall sensor toggles.
    LL_TIM_CC_SetUpdate(TIM1, LL_TIM_CCUPDATESOURCE_COMG_ONLY);
    
}

void enable_timers(){
    // We use the ADC1 and ADC2 in simulatenous mode to read the motor phase currents
    // around the PWM cycle up to down transition (when the ground mosfet is on). When 
    // the ADC is triggered in injected mode both ADC modules read the current for the 
    // U and V phases first then W and reference voltage second.
    // 
    // The ADC sample time is 20cycles, so the total sampling period is 20/12MHz = 1.67us.
    // 
    // Use the TIM1 channel 4 to generate an event a short time before the counter reaches
    // the auto reload value. This event triggers the ADC to read the motor phase currents.
    const uint16_t injected_conversion_start = pwm_base - sample_lead_time;
    LL_TIM_OC_SetCompareCH4(TIM1, injected_conversion_start);

    // It appears that the countermode is being ignored for the external ADC triggering.
    if (LL_TIM_GetCounterMode(TIM1) != LL_TIM_COUNTERMODE_CENTER_DOWN) error();
    // Invert the PWM mode of channel 4 instead so we emit a rising edge during up counting.
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);

    // Reinitialize the timers; reset the counters and update registers. Because the timers
    // are setup with preload registers the values we write to them are stored in shadow registers
    // and only applied when the update event occurs (to any individual PWM cycle consistent). We
    // need to generate an update event to force the timers to load the new values.
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM3);

    // Start the timers.
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM3);
    
    // Enable TIM1 channel 4 used to trigger the ADC.
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    
    // Enable the TIM1 outputs following a break or clock issue event.
    LL_TIM_EnableAllOutputs(TIM1);
    
    // Enable TIM2 channels 1 as the hall sensor timer between commutations.
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
}