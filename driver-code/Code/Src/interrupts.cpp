#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_motor.hpp"

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

// Interrupt Loop State
// ====================

// Electrical and position state
FullReadout readout = {};

// History of light readouts so that we can record every cycle for a short snapshot.
Readout readout_history[history_size] = {};

// Current write index.
size_t readout_history_write_index = 0;

// Current read index.
size_t readout_history_read_index = 0;


// Flag counters
// -------------

// Count the number of consecutive EMF detections.
int number_of_emf_detections = 0;

// Count for our belief that the rotor direction is incorrect. The EMF gives the axis
// of the rotor magnetic field, but we can have a 180 degree error in the direction
// until we identify the rotation direction.
int incorrect_direction_detections = 0;

// Track the noisiness of the EMF voltage.
int emf_voltage_variance = max_16bit;

// Track how many times we think our angle is correct.
int correct_angle_counter = 0;


// Motor driver state
// ------------------

// Currently active driver state (the full motor control state should be stored here).
DriverState driver_state = null_driver_state;

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
DriverState pending_state = null_driver_state;

// Currently active motor outputs.
MotorOutputs active_motor_outputs = breaking_motor_outputs;

// Keep a record of the previous output; the output switches mid cycle so we need to account for that.
MotorOutputs previous_motor_outputs = breaking_motor_outputs;


// Interrupt Data Interface
// ------------------------

FullReadout get_readout(){
    // Disable the ADC interrupt while we read the latest readout.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    // We should read from the circular buffer without disabling interrupts 
    // when we use a chip with more memory...
    FullReadout readout_copy = readout;
    NVIC_EnableIRQ(ADC1_2_IRQn);
    return readout_copy;
}

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

// (Private func) Push a readout to the history buffer.
static inline bool readout_history_push(Readout const& readout){
    if (readout_history_write_index >= history_size) return false;
    readout_history[readout_history_write_index] = readout;
    readout_history_write_index += 1;
    return true;
}

bool is_motor_safed(){
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

void set_angle(int16_t angle) {
    // Disable the ADC interrupt while we modify the readout angle.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    readout.angle = normalize_angle(angle);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

// Helper funcs
// ------------

// Combine the motor duty cycles into a single 32-bit value; because it barely fits.
static inline uint32_t encode_pwm_commands(MotorOutputs const & outputs){
    return (
        outputs.u_duty * pwm_base * pwm_base +
        outputs.v_duty * pwm_base +
        outputs.w_duty
    );
}

// Get the average between two motor outputs.
static inline MotorOutputs mid_motor_outputs(MotorOutputs const & a, MotorOutputs const & b){
    return MotorOutputs{
        .enable_flags = enable_flags_all,
        .u_duty = static_cast<uint16_t>((a.u_duty + b.u_duty) / 2),
        .v_duty = static_cast<uint16_t>((a.v_duty + b.v_duty) / 2),
        .w_duty = static_cast<uint16_t>((a.w_duty + b.w_duty) / 2)
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


// Interrupt handlers
// ------------------

// These functions are called by the autogenerated C code.

// Critical function!! 23KHz PWM cycle
// ===================================

// Process ADC readings for phase currents when the injected conversion is done.
void adc_interrupt_handler(){
    // Note: a single float assignment will cost us 5% of the CPU time (on STM32F103C8T6). We can't use floats...

    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    // Increment the readout number.
    const uint16_t readout_number = readout.readout_number + 1;

    // Start by reading the ADC conversion data
    // ----------------------------------------

    // Double check the ADC end of conversion flag was set.
    if (not LL_ADC_IsActiveFlag_JEOS(ADC1)) error();

    
    // U and W phases are measured at the same time, followed by V and the reference voltage.
    // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
    // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
    // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
    
    // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
    // The PWM period lasts 1/23.4KHz = 42.7us.

    const uint16_t ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3);

    // I wired the shunt resistors in the wrong way, so we need to flip the sign of the current readings.
    const int u_readout = -(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - ref_readout);
    // Flip the sign of V because we accidentally wired it the other way (the right way...). Oopsie doopsie.
    const int v_readout = +(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - ref_readout);
    const int w_readout = -(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - ref_readout);
        
    // Note that the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.

    const uint16_t instant_temperature = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    const uint16_t instant_vcc_voltage = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);


    // Do the data calculations
    // ------------------------


    // Average out the VCC voltage; it should be relatively stable so average to reduce our error.
    const int vcc_voltage = round_div(instant_vcc_voltage + readout.vcc_voltage * 3, 4);

    // Calculate our outputs on the motor phases.
    // If we managed to set the compare before the new cycle then it's the mid point; otherwise it's the previous outputs.
    const auto motor_outputs = readout.cycle_end_tick < pwm_base ? 
        previous_motor_outputs : 
        mid_motor_outputs(previous_motor_outputs, active_motor_outputs);

    // Remember the outputs for the next cycle.
    previous_motor_outputs = active_motor_outputs;

    // Calculate the readout differences; we need them to compute current divergence.

    const int u_readout_diff = u_readout - readout.u_readout;
    const int v_readout_diff = v_readout - readout.v_readout;
    const int w_readout_diff = w_readout - readout.w_readout;


    // Predict the position based on the previous state.
    const int predicted_angle = normalize_angle(readout.angle + readout.angular_speed / speed_fixed_point);

    // Get calibrated currents.

    const auto [u_current, v_current, w_current] = ThreePhase{
        u_readout * current_calibration.u_factor / current_calibration_fixed_point,
        v_readout * current_calibration.v_factor / current_calibration_fixed_point,
        w_readout * current_calibration.w_factor / current_calibration_fixed_point
    };

    // Get calibrated current divergence (the time unit is defined 1 per cycle).

    const auto [u_current_diff, v_current_diff, w_current_diff] = ThreePhase{
        u_readout_diff * current_calibration.u_factor / current_calibration_fixed_point,
        v_readout_diff * current_calibration.v_factor / current_calibration_fixed_point,
        w_readout_diff * current_calibration.w_factor / current_calibration_fixed_point
    };

    // Calibrated conversion factor between current divergence and phase inductance voltage.
    const int diff_to_voltage = phase_readout_diff_per_cycle_to_voltage * current_calibration.inductance_factor / current_calibration_fixed_point;

    // Calculate the voltage drop across the coil inductance.

    const int u_inductor_voltage = u_current_diff * diff_to_voltage / current_fixed_point;
    const int v_inductor_voltage = v_current_diff * diff_to_voltage / current_fixed_point;
    const int w_inductor_voltage = w_current_diff * diff_to_voltage / current_fixed_point;

    // Calculate the resistive voltage drop across the coil and MOSFET resistance.
    
    const int u_resistive_voltage = u_current * phase_current_to_voltage / current_fixed_point;
    const int v_resistive_voltage = v_current * phase_current_to_voltage / current_fixed_point;
    const int w_resistive_voltage = w_current * phase_current_to_voltage / current_fixed_point;

    // Calculate the driven phase voltages from our PWM settings and the VCC voltage.

    const auto [u_drive_voltage, v_drive_voltage, w_drive_voltage] = adjust_to_sum_zero(ThreePhase{
        round_div(motor_outputs.u_duty * vcc_voltage, pwm_base),
        round_div(motor_outputs.v_duty * vcc_voltage, pwm_base),
        round_div(motor_outputs.w_duty * vcc_voltage, pwm_base)
    });

    // Infer the back EMF voltages for each phase.
    // 
    // Calculate the EMF voltage as the remainder after subtracting the electric circuit voltages.
    // By Kirchoffs laws the total voltage must sum to 0.

    const int u_emf_voltage = -(u_drive_voltage - u_resistive_voltage - u_inductor_voltage);
    const int v_emf_voltage = -(v_drive_voltage - v_resistive_voltage - v_inductor_voltage);
    const int w_emf_voltage = -(w_drive_voltage - w_resistive_voltage - w_inductor_voltage);

    
    // Calculate the park transformed currents and voltages
    // 
    // Use gradient descent to estimate the inductor current angle. We don't have compute to
    // calculate the angle with atan2, so we treat it as an optimization problem over multiple
    // cycles. With cycles at 23KHz, we converge quickly, especially at high current values.
    // 
    // Since the motor induced currents are smooth, we should also maintain smooth output to
    // help our lack of compute. If both driving and emf voltages vary smoothly the current
    // angle must also vary smoothly.
    //
    // Exponentially average the values below to reduce noise, 1 : 3 parts coorresponds to 150us half life
    // at our cycle frequency.
    
    // First alias the trig functions based on the predicted rotor angle.

    const int u_cos = get_cos(predicted_angle);
    const int v_cos = get_cos(predicted_angle - third_circle);
    const int w_cos = get_cos(predicted_angle - two_thirds_circle);
    const int u_neg_sin = -get_sin(predicted_angle);
    const int v_neg_sin = -get_sin(predicted_angle - third_circle);
    const int w_neg_sin = -get_sin(predicted_angle - two_thirds_circle);

    // Park transform the currents and voltages.

    const int alpha_current = (
        u_current * u_cos +
        v_current * v_cos +
        w_current * w_cos
    ) / angle_base;

    const int beta_current = (
        u_current * u_neg_sin +
        v_current * v_neg_sin +
        w_current * w_neg_sin
    ) / angle_base;

    const int alpha_emf_voltage = (
        u_emf_voltage * u_cos +
        v_emf_voltage * v_cos +
        w_emf_voltage * w_cos
    ) / angle_base;

    const int beta_emf_voltage = (
        u_emf_voltage * u_neg_sin +
        v_emf_voltage * v_neg_sin +
        w_emf_voltage * w_neg_sin
    ) / angle_base;


    // TODO: reimplement drive modes.
    // TODO: allows sporading missing EMF or current readings
    // TODO: track EMF fix since startup, ! definitely have a button to set random angle so we can test it
    // TODO: fix the direction flipping

    // Current angle calculation
    // -------------------------

    const bool current_detected = square(alpha_current) + square(beta_current) > 16;

    const int inductor_angle = normalize_angle(predicted_angle + funky_atan2(beta_current, alpha_current));


    // Back EMF observer
    // -----------------

    // Get the angle measured from EMF relative to the predicted rotor angle.
    const int emf_angle_error = funky_atan2(alpha_emf_voltage, -beta_emf_voltage);

    // Calculate the emf voltage as a rotation of the beta voltage that zeroes out the alpha component.
    const int instant_emf_voltage_magnitude = faster_abs(
        -(get_cos(emf_angle_error) * beta_emf_voltage - get_sin(emf_angle_error) * alpha_emf_voltage) / angle_base
    );

    const int emf_voltage_error = instant_emf_voltage_magnitude - readout.emf_voltage_magnitude;

    const int emf_voltage_magnitude = readout.emf_voltage_magnitude + signed_ceil_div(emf_voltage_error, 16);

    const int square_emf_voltage_error = square(emf_voltage_error);

    emf_voltage_variance = min(max_16bit, 1 + (15 * emf_voltage_variance + square_emf_voltage_error) / 16);

    const bool emf_is_above_threshold = emf_voltage_magnitude > control_parameters.min_emf_voltage;
    
    const bool emf_is_above_noise = square(emf_voltage_magnitude) > 9 * emf_voltage_variance;

    const bool emf_is_accurate = square_emf_voltage_error < 4 * emf_voltage_variance;

    // Check if the emf voltage is away from zero with enough confidence.
    const bool emf_detected = emf_is_above_noise and emf_is_accurate;

    number_of_emf_detections = clip_to(0, 64, number_of_emf_detections + (emf_detected ? +1 : -1));

    // Let the angle adjust a few steps before using the diff to compute the speed; our initial guess starts
    // at an arbitrary position so the apparent acceleration is just the angle converging to the correct value.
    const bool compute_speed = emf_detected and (number_of_emf_detections >= 16);

    // Declare that we have an emf reading after enough detections.
    const bool emf_fix = number_of_emf_detections >= 32;

    // Memoize the abs of the angular speed as we will use it multiple times.
    const int abs_angular_speed = faster_abs(readout.angular_speed);

    // Declare we have movement if we're over the threshold speed.
    const bool movement_fix = abs_angular_speed >= control_parameters.min_emf_speed;

    // Flag for when the EMF angle and speed are both valid for other calculations.
    const bool emf_and_movement_fix = emf_fix and movement_fix;

    // Check if we have the incorrect rotor angle by checking if the beta EMF voltage has opposite sign to the angular speed.
    const bool incorrect_rotor_angle_detected = emf_detected and emf_and_movement_fix and (beta_emf_voltage * readout.angular_speed > 0);

    // Keep track in a counter.
    incorrect_direction_detections = clip_to(0, 32, incorrect_direction_detections + (incorrect_rotor_angle_detected ? +1 : -1));

    // Set the flag for immininent rotor correction, we need to set it over multiple cycles otherwise it might be missed.
    const bool rotor_direction_flip_imminent = incorrect_direction_detections >= 24;

    // Flip the rotor if we have reached the threshold number of incorrect detections.
    const bool rotor_direction_flip = incorrect_direction_detections >= 32;

    // Reset the incorrect direction counter on flip.
    incorrect_direction_detections = (not rotor_direction_flip) * incorrect_direction_detections;

    // Track how many times we think our rotor angle is correct.
    correct_angle_counter = clip_to(0, 64, correct_angle_counter + (emf_and_movement_fix and (incorrect_direction_detections == 0) ? +1 : -1));

    // Declare the angle to be correct after a threshold certainty.
    const bool angle_fix = correct_angle_counter >= 32;


    // If the angle error is between -90 and +90 degrees, use it directly otherwise use the mirror angle.
    const int prediction_error = angle_or_mirror(emf_angle_error);

    // Calculate the angle adjustment error using the parametrized gain.
    const int angle_error = emf_detected * signed_ceil_div(
        prediction_error * control_parameters.rotor_angle_ki,
        control_parameters_fixed_point
    );

    // Adjust the angle prediction based on the EMF voltage. The EMF voltage must be have opposite sign to the
    // the speed; if it doesn't then we must have fixed on the opposite side of the rotor; rotate 180 to correct it.
    const int updated_angle = normalize_angle(predicted_angle + angle_error + rotor_direction_flip * half_circle);


    // Calculate the new speed based on the angle adjustment.
    // 
    // Note that the angle change is relative to the current speed because of the prediction step.
    const int speed_error = (
        // If we have enough EMF detections, adjust the speed according to the prediction error.
        compute_speed ? signed_ceil_div(
            speed_fixed_point * prediction_error * control_parameters.rotor_angular_speed_ki, 
            control_parameters_fixed_point) :    
        // Maintain speed if we have an EMF reading, even if noisy.
        emf_is_above_threshold ? 0 :
        // Otherwise quickly decay the speed towards zero.
        -signed_ceil_div(readout.angular_speed, 32)
    );
    
    const int updated_speed = readout.angular_speed + speed_error;

    // Calculate the acceleration based on the speed change.
    // 
    // The new speed isn't predicted so we need to diff to the previous acceleration to get the error.
    const int acceleration_error = signed_ceil_div(
        (speed_error * acceleration_fixed_point - readout.rotor_acceleration) * control_parameters.rotor_acceleration_ki,
        control_parameters_fixed_point
    );
    const int updated_acceleration = readout.rotor_acceleration + acceleration_error;



    // Calculate the motor constant
    // ----------------------------

    const int predicted_emf_voltage = abs_angular_speed * readout.motor_constant / emf_motor_constant_conversion;

    const bool compute_motor_constant = emf_and_movement_fix and emf_is_above_threshold;

    const int motor_constant_error = compute_motor_constant * (emf_voltage_magnitude - predicted_emf_voltage);
    const int motor_constant = (
        readout.motor_constant +
        signed_ceil_div(motor_constant_error * control_parameters.motor_constant_ki, control_parameters_fixed_point)
    );


    // Calculate the power values using the phase currents and voltages.
    
    const int resistive_power = (
        u_current * u_resistive_voltage + 
        v_current * v_resistive_voltage + 
        w_current * w_resistive_voltage
    ) / voltage_current_div_power_fixed_point;

    const int inductive_power = (
        u_current * u_inductor_voltage + 
        v_current * v_inductor_voltage + 
        w_current * w_inductor_voltage
    ) / voltage_current_div_power_fixed_point;

    // Nope, the phase currents lie! We should assume the motor is behaving rationally and
    // smoothly while our measurements are noisy and lagged at high speed. Theoretically
    // the equations below give the correct power values, but it's actually better to
    // compute power from the DQ0 transformed values with some adjustments.
    // 
    // const int wrong_emf_power = -(
    //     u_current * u_emf_voltage + 
    //     v_current * v_emf_voltage + 
    //     w_current * w_emf_voltage
    // ) / voltage_current_div_power_fixed_point;

    // const int wrong_total_power = -(
    //     u_current * u_drive_voltage + 
    //     v_current * v_drive_voltage + 
    //     w_current * w_drive_voltage
    // ) / voltage_current_div_power_fixed_point;

    // By assuming the motor moves smoothly we claim the beta current is closer to the
    // measured magnitude of the DQ0 current. We then adjust our beta current using
    // a Taylor expansion for small alpha current. The system uses the real beta current
    // not what we measure; thus we compute the emf power using our massaged DQ0 values.
    const int emf_power = -beta_current * beta_emf_voltage / dq0_to_power_fixed_point;

    // Compute the real total power used from the balance of powers.
    // 
    // Resistive power is the power dissipated in the motor coils and MOSFETs.
    // EMF power is the power transferred into the rotor movement, driving the motor.
    // Inductive power is the power transfered to the motor inductance.
    // The total power is the power transferred to the battery.
    // 
    // The balance of all powers must be zero assuming no other source or sink of power. Thus
    // we can compute the total power from the others; mostly determined by EMF. The resistive
    // power is quite reliable and inductive_power is very small.
    const int total_power = -(resistive_power + emf_power + inductive_power);




    // Get hall sensor state
    // ---------------------
    // 
    // For debugging purposes...

    // Read new data from the hall sensors.
    const uint8_t hall_state = read_hall_sensors_state();


    // Write the latest readout data
    // -----------------------------
    // 
    // Must be updated before the motor pwm calculation!

    readout.readout_number = readout_number;
    
    readout.pwm_commands = encode_pwm_commands(motor_outputs);
        
    readout.state_flags = (
        (hall_state << hall_state_bit_offset) |
        (emf_fix << emf_fix_bit_offset) |
        (emf_detected << emf_detected_bit_offset) |
        (current_detected << current_detected_bit_offset) |
        (angle_fix << angle_fix_bit_offset) |
        (incorrect_rotor_angle_detected << incorrect_rotor_angle_bit_offset) |
        (rotor_direction_flip_imminent << rotor_direction_flip_imminent_bit_offset)
    );

    readout.ref_readout = ref_readout;

    readout.u_readout = u_readout;
    readout.v_readout = v_readout;
    readout.w_readout = w_readout;

    readout.u_readout_diff = u_readout_diff;
    readout.v_readout_diff = v_readout_diff;
    readout.w_readout_diff = w_readout_diff;
    
    readout.temperature = round_div(instant_temperature * 1 + readout.temperature * 15, 16);

    readout.instant_vcc_voltage = instant_vcc_voltage;
    readout.vcc_voltage = vcc_voltage;

    readout.angle = updated_angle;
    readout.angular_speed = updated_speed;

    readout.alpha_current = alpha_current;
    readout.beta_current = beta_current;
    readout.alpha_emf_voltage = alpha_emf_voltage;
    readout.beta_emf_voltage = beta_emf_voltage;
    
    readout.total_power = total_power;
    readout.resistive_power = resistive_power;
    readout.emf_power = emf_power;
    readout.inductive_power = inductive_power;
    

    readout.motor_constant = motor_constant;
    readout.inductor_angle = inductor_angle;

    readout.emf_voltage_magnitude = emf_voltage_magnitude;
    readout.rotor_acceleration = updated_acceleration;

    readout.angle_error = angle_error;
    readout.phase_resistance = 0;

    readout.emf_voltage_variance = emf_voltage_variance;
    readout.phase_inductance = 0;
    
    readout.debug_1 = driver_state.lead_angle_control;
    readout.debug_2 = 0;
    

    
    // Calculate motor outputs
    // -----------------------

    // Setup the new state if we were commanded by the main loop.
    if (new_pending_state) {
        driver_state = setup_driver_state(pending_state, readout);
        new_pending_state = false;
    }

    // Update the motor controls using the readout data.
    update_motor_control(active_motor_outputs, driver_state, readout);


    // Try to write the latest readout if there's space.
    readout_history_push(readout);


    // Set Motor Outputs!!
    // -------------------

    // Send the command to the timer compare registers. Set the registers close to when cycle_end_tick 
    // is set so we can properly track the value for the next cycle. There's a half cycle delay if
    // we set the output registers too late in the cycle.
    set_motor_outputs(active_motor_outputs);

    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    // Clear the ADC end of conversion flag so we're ready for the next conversion.
    LL_ADC_ClearFlag_JEOS(ADC1);
}


void tim1_update_interrupt_handler(){
    // We shouldn't trigger this, but including for documentation.
    error();
    
    // Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
    
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


#pragma GCC push_options
#pragma GCC optimize ("O0")

// Write only variable, for funsies.
volatile int write_only = 0;

void initialize_angle_tracking(){
    // Load the phase and sin tables into memory.
    for (int i = 0; i < angle_base; i += 1) {
        write_only = get_phase_pwm(i);
        write_only = get_sin(i);
    }
}
#pragma GCC pop_options
