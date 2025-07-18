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

// History of shorthand readouts that we record while keeping the comms quiet (so we don't miss any).
Readout readout_history[history_size] = {};

// Current write index.
size_t readout_history_write_index = 0;

// Current read index.
size_t readout_history_read_index = 0;


// Position Tracking & Observers
// -----------------------------

// Best estimate of the **electric** angle and speed (that is the angle between the coil 
// U and the magnetic North). The actual position of the output depends on the number of
// pole pairs, and the slot triplets, and the mechanical gear ratio.

// All tracked observers, including the rotor and inductor positions, resistances, inductances, etc.
Observers observers = {
    .rotor_angle = {},
    .rotor_angular_speed = {},
    .rotor_acceleration = {},
    .motor_constant = {},
    .resistance = {},
    .inductance = {},
};

// Count the number of consecutive EMF detections; we get good statistics after a threshold number.
size_t consecutive_emf_detections = 0;
size_t incorrect_direction_detections = 0;

size_t consecutive_current_detections = 0;


// Motor driver state
// ------------------

// Currently active driver state.
DriverState driver_state = DriverState::OFF;

// Currently active driver parameters (the full motor control state should be stored here).
DriverParameters driver_parameters = {};

// Must be volatile as it's the interaction flag between main loop and the interrupt handler.
// 
// The main loop will set pending_state and pending_parameters to the user command, but only
// when the pending_state is DriverState::NO_CHANGE. The interrupt handler will copy the
// pending_state and pending_parameters to the active state variables, and reset the pending_state.
// 
// Volatile will prevent the compiler from optimizing out the read/write operations to this variable.
// In our case, the main loop will read this variable in a hot while loop that has no side effects, 
// expecting the variable to be set by the interrupt handler. The compiler *will* optimize out the
// while loop unless we mark the variable as volatile.
volatile DriverState pending_state = DriverState::NO_CHANGE;

// Parameters for the new driver state.
DriverParameters pending_parameters = {};

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
    return (driver_state == DriverState::OFF) || (driver_state == DriverState::FREEWHEEL);
}

void set_motor_command(DriverState const& state, DriverParameters const& parameters){
    // Don't override a pending command if the interrupt loop didn't copy it to active.
    while (pending_state != DriverState::NO_CHANGE) continue;

    pending_state = state;
    pending_parameters = parameters;
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
    // Note: a single float assignment will costs us 5% of the CPU time (on STM32F103C8T6). We can't use floats...

    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    


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
    observers.rotor_angle.value += observers.rotor_angular_speed.value / speed_fixed_point;

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

    const int u_cos = get_cos(observers.rotor_angle.value);
    const int v_cos = get_cos(observers.rotor_angle.value - third_circle);
    const int w_cos = get_cos(observers.rotor_angle.value - two_thirds_circle);
    const int u_neg_sin = -get_sin(observers.rotor_angle.value);
    const int v_neg_sin = -get_sin(observers.rotor_angle.value - third_circle);
    const int w_neg_sin = -get_sin(observers.rotor_angle.value - two_thirds_circle);

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
    // TODO: better command sliders for the drive modes in the UI.

    // Current angle calculation
    // -------------------------

    const bool current_detected = square(alpha_current) + square(beta_current) > 16;

    const int inductor_angle = normalize_angle(observers.rotor_angle.value + funky_atan2(beta_current, alpha_current));

    consecutive_current_detections = current_detected * (consecutive_current_detections + 1);

    const bool current_fix = consecutive_current_detections > 4;


    // Back EMF observer
    // -----------------

    const int rotor_angle_error = funky_atan2(alpha_emf_voltage, -beta_emf_voltage);

    // Calculate the emf voltage as a rotation of the beta voltage. This should always be positive, but let's make sure of it.
    const int emf_voltage_magnitude = faster_abs(
        -(get_cos(rotor_angle_error) * beta_emf_voltage - get_sin(rotor_angle_error) * alpha_emf_voltage) / angle_base
    );


    // Check if the emf voltage is away from zero with enough confidence.
    const bool emf_detected = emf_voltage_magnitude > emf_min_voltage;

    // The EMF voltage gives us a noisy but accurate estimate of the rotor magnetic angle;
    // use it to update the position between hall sensor toggles.
    if (emf_detected) {
        // If the angle error is between -90 and +90 degrees, use it directly otherwise use the mirror angle.
        const int prediction_error = angle_or_mirror(rotor_angle_error);

        observers.rotor_angle.error = signed_ceil_div(
            prediction_error * control_parameters.rotor_angle_ki,
            control_parameters_fixed_point
        );
        observers.rotor_angle.value = normalize_angle(observers.rotor_angle.value + observers.rotor_angle.error);

        const bool compute_speed = consecutive_emf_detections > 4;

        // Calculate the new speed based on the angle adjustment.
        // 
        // Note that the angle change is relative to the current speed because of the prediction step.
        observers.rotor_angular_speed.error = compute_speed * speed_fixed_point * signed_ceil_div(
            prediction_error * control_parameters.rotor_angular_speed_ki, 
            control_parameters_fixed_point
        );
        observers.rotor_angular_speed.value += observers.rotor_angular_speed.error;

        // Calculate the acceleration based on the speed change.
        // 
        // The new speed isn't predicted so we need to diff to the previous acceleration to get the error.
        observers.rotor_acceleration.error = observers.rotor_angular_speed.error * acceleration_fixed_point - observers.rotor_acceleration.value;
        observers.rotor_acceleration.value += observers.rotor_acceleration.error * control_parameters.rotor_acceleration_ki / control_parameters_fixed_point;
    } else {
        observers.rotor_angle.error = 0;
        observers.rotor_angle.value = observers.rotor_angle.value;

        observers.rotor_angular_speed.error = -signed_ceil_div(observers.rotor_angular_speed.value, 256);
        observers.rotor_angular_speed.value += observers.rotor_angular_speed.error;

        observers.rotor_acceleration.value = 0;
        observers.rotor_acceleration.error = 0;
    }



    const bool incorrect_rotor_angle = beta_emf_voltage * observers.rotor_angular_speed.value > emf_direction_threshold;

    incorrect_direction_detections = incorrect_rotor_angle * (incorrect_direction_detections + 1);

    consecutive_emf_detections = emf_detected * (consecutive_emf_detections + 1);

    // Beta voltage and the speed must have opposite signs; fix if they don't.
    if (incorrect_direction_detections > 8) {
        // We detected the rotor in the wrong quadrant; it's the mirrored angle.
        observers.rotor_angle.value = normalize_angle(observers.rotor_angle.value + half_circle);
        // Reset the incorrect detection counter.
        incorrect_direction_detections = 0;
        consecutive_emf_detections = 0;
    }

    const bool emf_fix = consecutive_emf_detections > 16;


    // Note use rotor speed after correction!

    const int predicted_emf_voltage = faster_abs(observers.rotor_angular_speed.value) * observers.motor_constant.value / emf_motor_constant_conversion;

    observers.motor_constant.error = emf_fix * (emf_voltage_magnitude - predicted_emf_voltage);
    observers.motor_constant.value += (
        observers.motor_constant.error * control_parameters.motor_constant_ki / control_parameters_fixed_point
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

    readout.pwm_commands = encode_pwm_commands(motor_outputs);

    // Advance the readout number.
    readout.readout_number = readout.readout_number + 1;
        
    readout.state_flags = (
        (hall_state << hall_state_bit_offset) |
        (emf_fix << emf_fix_bit_offset) |
        (emf_detected << emf_detected_bit_offset) |
        (current_fix << current_fix_bit_offset) |
        (current_detected << current_detected_bit_offset)
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

    readout.angle = observers.rotor_angle.value;
    readout.angular_speed = observers.rotor_angular_speed.value;
    
    readout.alpha_current = alpha_current;
    readout.beta_current = beta_current;
    readout.alpha_emf_voltage = alpha_emf_voltage;
    readout.beta_emf_voltage = beta_emf_voltage;

    // readout.u_debug = ;
    // readout.v_debug = ;
    // readout.w_debug = ;
    readout.motor_constant = observers.motor_constant.value;

    readout.total_power = total_power;
    readout.resistive_power = resistive_power;
    readout.emf_power = emf_power;
    readout.inductive_power = inductive_power;

    readout.inductor_angle = inductor_angle;
    readout.emf_voltage_magnitude = emf_voltage_magnitude;

    readout.angle_error = observers.rotor_angle.error;
    readout.angular_speed_error = observers.rotor_angular_speed.error;
    
    readout.rotor_acceleration = observers.rotor_acceleration.value;
    readout.rotor_acceleration_error = observers.rotor_acceleration.error;

    
    // Calculate motor outputs
    // -----------------------

    // Update the motor controls using the readout data. Note that it also modifies the driver_state, driver_parameters, and pid_state.
    const bool critical_error = update_motor_control(
        active_motor_outputs, driver_state, driver_parameters,
        pending_state, pending_parameters, readout
    );

    // Always reset the pending state to be ready for the next command.
    pending_state = DriverState::NO_CHANGE;

    if (critical_error) error();


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
    for (int i = 0; i < angle_base; i += 8) {
        write_only = get_phase_pwm(i);
        write_only = get_sin(i);
    }
}
#pragma GCC pop_options
