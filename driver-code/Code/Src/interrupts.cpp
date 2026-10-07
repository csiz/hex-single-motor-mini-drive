#include "interrupts.hpp"

#include "hex_mini_drive_interface.hpp"

#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "io.hpp"
#include "constants.hpp"
#include "dq0_transform.hpp"
#include "circular_buffer.hpp"

// TODO: need to fix this...
// The interrupts must not enter the error handler!
// 
// The error handle will block forever, however we must safe the motor no matter what. The interrupt loop
// will always timeout any command and return to a safe state if it doesn't receive new commands from the
// main app loop.
// 
// Do not: #include "error_handler.hpp"


#include <cmath>
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


// Interrupt Data Interface
// ------------------------

// Medium length data recording, taken continuously from the interrupt handler.
CircularQueue<hex_mini_drive::Readout, hex_mini_drive::HISTORY_SIZE> readout_history_queue = {};

// Command queue, the interrupt loop will switch to executing the latest set of commands from this queue.
// ! Note, we should keep it twice the size of the maximum schedule + 1. This way we can load the next
// schedule while the first is executing, but also we will have 1 more space to write an emergency break
// command! The break command will also lead to the interrupt loop freeing up space in the command queue.
CircularQueue<hex_mini_drive::DriveState, 25> drive_state_queue = {};

size_t drive_states_scheduled = 0;

// Initialize the loop control parameters and the calibration data. Either load 
// them from the flash or use the defaults.

hex_mini_drive::CurrentCalibration current_calibration = get_current_calibration();

hex_mini_drive::ControlParameters control_parameters = get_control_parameters();


// Interrupt Loop State
// --------------------

// Electrical and position state
hex_mini_drive::Readout readout = {};

// Angle offsetting from the main loop or via commands; it will not influence speed.
volatile int32_t external_angle_offset = 0;

// Also offset the current rotations counter.
volatile int32_t external_rotations_offset = 0;


// Additional state
// ----------------

// Track how many times we think our angle is correct.
int32_t correct_angle_counter = 0;

// Settings for the 3 phase MOSFET PWM drivers.
MotorOutputs motor_outputs = breaking_motor_outputs;

// Our outputs are delayed 1 cycle; store the previous outputs here before we use them.
MotorOutputs previous_motor_outputs = breaking_motor_outputs;

// Currently active driver state (the full motor control state should be stored here).
hex_mini_drive::DriveState drive_state = breaking_drive_state;



bool is_motor_safed(){
    // Consider both motor breaking and freewheeling as safe states.
    return (drive_state.pwm_value == 0.f);
}

void set_angle_offset(int32_t angle_offset) {
    external_angle_offset = angle_offset;
}

void set_rotations_offset(int32_t rotations_offset) {
    external_rotations_offset = rotations_offset;
}

// Critical functions!! 23KHz PWM cycle
// ====================================

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

// Motor control helper
// --------------------

// Set the motor outputs to the specified active_pwm and active_angle.
static inline MotorOutputs compute_motor_outputs(
    int32_t const& angle,
    float const& pwm_value,
    float const& live_max_pwm
) {
    // We need a single comparison and we can then use arithmetic math instead of branching conditionals.
    const bool negative_pwm = pwm_value < 0.f;

    // The PWM counter value must be positive, we use the sign to determine the direction.
    const float abs_pwm = min(live_max_pwm, negative_pwm ? -pwm_value : pwm_value);

    // Use the active angle or flip it depending on the sign of the PWM.
    const int32_t adjusted_angle = angle + negative_pwm * half_circle;

    // Get the voltage for the three phases from the waveform table.
    const float voltage_phase_u = get_phase_pwm(adjusted_angle);
    const float voltage_phase_v = get_phase_pwm(adjusted_angle - third_circle);
    const float voltage_phase_w = get_phase_pwm(adjusted_angle - neg_third_circle);

    return MotorOutputs{
        .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm)
    };
}

// ADC readings and calculation loop
// ---------------------------------

// Process ADC readings for phase currents when the injected conversion is done.
void ADC1_2_IRQHandler(void){
    // Note: a single float assignment will cost us 5% of the CPU time (on STM32F103C8T6). We can't use floats...

    // Check what time it is on the PWM cycle.
    readout.cycle_start_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));
    
    
    // Start by reading sensor data
    // ----------------------------
    
    // Double check the ADC end of conversion flag was set for both ADCs.
    if (not (LL_ADC_IsActiveFlag_JEOS(ADC1) and LL_ADC_IsActiveFlag_JEOS(ADC2))) {
        // If we only get one set of readings it may be a startup timing issue, ignore it and
        // clear both flags to reset the ADCs.
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC2);
        return;
    }
    
    // Get the ADC readings for the phase currents, temperature, and VCC voltage.
    const ADCReadings adc_readings = read_adc_values();

    // Read new data from the hall sensors.
    const uint8_t hall_state = read_hall_sensors_state();
    
    // Increment the readout number.
    const uint16_t readout_number = readout.readout_number + 1;


    // Do the data calculations
    // ------------------------
    // 
    // !!! The inductor voltages cannot be neglected despite being very noisy... curses... They also cannot be 
    // filtered as it introduces additional phase lag and it's detrimental to add the lag to all the signals to
    // keep the math consistent. They can't even be filtered conceptually because some short-term variations are
    // real and similar to the fake jumps from noise. The noise can be determined in retrospect, but not live.
    // 
    // We can use an exponential filter on the current diffs and the equations remain invariant as long as we use
    // the exact same filter for drive_voltages and currents. However this filter introduces a phase lag and that
    // appears to be more detrimental than just ignoring the current diffs and inductor voltages altogether.
    // 
    // I've attempted to use the finite differences approach `diff(x) = ((x[n] - x[n-1]) + (x[n+1] - x[n])) / (2 * dt)`
    // but it doesn't work as well as exponential filtering.


    // Average the temperature readings since we are sampling quicker than the manufacturer indicates.
    // We can't extend the sampling time longer than it is set at the moment (about half the recommendation),
    // so we have to massage the readings for noise. Temperature varies slowly anyway.
    const float temperature = (adc_readings.temp_readout * 0.25f + readout.temperature * 0.75f);

    // Average out the VCC voltage; it should be relatively stable so we average to reduce our error.
    const float vcc_voltage = (adc_readings.vcc_readout * voltage_conversion * 0.25f + readout.vcc_voltage * 0.75f);

    // Calculate calibrated currents.
    // 
    // We need to flip the sign of the current readings. Our convention is to have settle on positive
    // current when we apply a positive PWM duty cycle to each respective phase.
    // 
    // Note that the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.
    const ThreePhase currents{
        -static_cast<float>(adc_readings.u_readout - adc_readings.ref_readout) * adc_to_current_units - current_calibration.u_current_zero,
        -static_cast<float>(adc_readings.v_readout - adc_readings.ref_readout) * adc_to_current_units - current_calibration.v_current_zero,
        -static_cast<float>(adc_readings.w_readout - adc_readings.ref_readout) * adc_to_current_units - current_calibration.w_current_zero
    };


    // Precompute the conversion coeffiecient for the motor duties.
    const float half_pwm_to_vcc = vcc_voltage * half_pwm_base_inverse;

    // Calculate our outputs on the motor phases. The outputs kick in halfway through the PWM cycle,
    // so we average the previous and current outputs to get the effective output for this cycle.
    // 
    // Calculate the driven phase voltages from our PWM settings and the VCC voltage. We adjust our voltages
    // such that the 0 point corresponds to the voltage at the connection point of the three phases. The
    // motor stator coils are usually connected together by the manufacturer for a star configuration motor.
    const ThreePhase drive_voltages = {
        (motor_outputs.u_duty + previous_motor_outputs.u_duty) * half_pwm_to_vcc,
        (motor_outputs.v_duty + previous_motor_outputs.v_duty) * half_pwm_to_vcc,
        (motor_outputs.w_duty + previous_motor_outputs.w_duty) * half_pwm_to_vcc
    };

    // Store the active motor outputs for the next cycle.
    previous_motor_outputs = motor_outputs;


    // Predict the position; keeping track of fractional angles at the same resolution as
    // the speed. By our definition the time unit is 1 per cycle; so the angle spanned by 
    // the rotor is exactly the angular speed.
    const int32_t predicted_angle = readout.angle + static_cast<int32_t>(readout.angular_speed);

    // Switching to DQ0 Frame
    // ----------------------
    // 
    // Calculate the park transformed currents and voltages: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
    // 
    // We reduce the number of operations and allow for more uses of constants if we 
    // immediately transform to the dq0 frame at the fixed angle 0. We can always
    // rotate the frame afterwards using a 2D rotation matrix with just 4 elements.

    // Transform the measured currents into the DQ0 frame at the fixed angle 0.
    const auto [direct_current, quadrature_current] = dq0_transform(currents);

    // Invoke the CORDIC engine to compute atan2 and magnitude using the phase function.
    set_cordic(direct_current, quadrature_current);

    // Get the common mode current. It should be 0 in theory, but of course it is not in practice...
    const float zero_current = (std::get<0>(currents) + std::get<1>(currents) + std::get<2>(currents)) * one_third;

    // And transform the drive voltages too.
    const auto [direct_drive_voltage, quadrature_drive_voltage] = dq0_transform(drive_voltages);


    // Current angle calculation
    // -------------------------
    // 
    // We calculate the angle of the current vector that is running through the motor coils.
    // 
    // In our convention the inductors driven with positive current form a south pole that attracts
    // the north pole of the rotor.

    // Calculate the angle at which the current is running on the motor coils. The angle offset is
    // with respect to the predicted angle as that was the angle used in the park transform.
    const auto [instant_current_angle, instant_current_magnitude] = get_cordic();
    
    // The current measurements have a low noise floor, but it's not 0.
    const bool current_detected = instant_current_magnitude > control_parameters.current_measurement_minimum;

    const int32_t predicted_current_angle = predicted_angle + readout.current_angle_offset;

    const int32_t current_angle_error = instant_current_angle - predicted_current_angle;

    const int32_t current_angle = predicted_current_angle + static_cast<int32_t>(current_angle_error * control_parameters.current_angle_ki);

    const int32_t current_angle_offset = current_angle - predicted_angle;

    const float current_magnitude = readout.current_magnitude + (instant_current_magnitude - readout.current_magnitude) * control_parameters.current_magnitude_ki;
    
    // Using DQ0 coordinates
    // ---------------------
    
    // Calculate the resistive voltage drop across the coil and MOSFET resistance.
    const float direct_resistive_voltage = direct_current * current_calibration.resistance;
    const float quadrature_resistive_voltage = quadrature_current * current_calibration.resistance;

    // Calculate the differential of the currents; this will be the real measurement of our model's prediction.
    const float direct_current_diff = direct_current - readout.direct_current;
    const float quadrature_current_diff = quadrature_current - readout.quadrature_current;

    const float direct_diff = direct_current_diff * current_diff_to_voltage_units;
    const float quadrature_diff = quadrature_current_diff * current_diff_to_voltage_units;

    // The saliency effect is due to the iron between magnets so it occurs twice per 
    // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.
    // For simplicity's sake we keep the saliency_angle_offset as already doubled.
    const int32_t saliency_angle = 2*predicted_angle + current_calibration.saliency_angle_offset;

    const float cos_saliency = get_cos(saliency_angle);
    const float sin_saliency = get_sin(saliency_angle);

    const float direct_inductance_bias_voltage = current_calibration.inductance_bias * (
        -direct_diff * cos_saliency 
        -quadrature_diff * sin_saliency
    );
    const float quadrature_inductance_bias_voltage = current_calibration.inductance_bias * (
        -direct_diff * sin_saliency
        +quadrature_diff * cos_saliency
    );

    const float direct_inductance_base_voltage = direct_diff * current_calibration.inductance;
    const float quadrature_inductance_base_voltage = quadrature_diff * current_calibration.inductance;

    const float direct_inductor_voltage = direct_inductance_base_voltage + direct_inductance_bias_voltage;
    const float quadrature_inductor_voltage = quadrature_inductance_base_voltage + quadrature_inductance_bias_voltage;

    const float measured_direct_emf_voltage = direct_inductor_voltage + direct_resistive_voltage - direct_drive_voltage;
    const float measured_quadrature_emf_voltage = quadrature_inductor_voltage + quadrature_resistive_voltage - quadrature_drive_voltage;

    // Send the emf values to the CORDIC asap.
    set_cordic(measured_direct_emf_voltage, measured_quadrature_emf_voltage);


    // Back EMF angle observer
    // -----------------------
    
    // Calculate the predicted EMF angle.
    const int32_t predicted_emf_voltage_angle = readout.emf_voltage_angle + static_cast<int32_t>(readout.angular_speed);

    // Get the angle measured from EMF relative to the predicted rotor angle.
    const auto [measured_emf_voltage_angle, measured_emf_voltage_magnitude] = get_cordic();

    const int32_t emf_angle_error = measured_emf_voltage_angle - predicted_emf_voltage_angle;

    const int32_t emf_angle_error_plus_quarter = emf_angle_error + quarter_circle;

    const int32_t emf_angle_flip = emf_angle_error_plus_quarter & most_negative_angle;

    // Compute the angle error to the emf axis.
    // 
    // Branching if statements are easier to understand, but they do take 40 cycles extra, so we end up doing
    // it the bit bang "clever" way. Anyway, the point is we divide the angle space in 2 by and-ing the maximum
    // positive angle and then we center it on 0 by subtracting a quarter circle.
    const int32_t emf_axis_error = (emf_angle_error_plus_quarter & most_positive_angle) - quarter_circle;


    const int32_t emf_angle_adjustment = emf_axis_error * control_parameters.emf_angle_ki;

    // Update the new emf_voltage_angle. If the emf changes sign, we will also immediately change sign.
    // But we slowly trace to the new angle by integrating the error over time using the control parameter.
    const int32_t emf_voltage_angle = (
        predicted_emf_voltage_angle + 
        emf_angle_flip + 
        emf_angle_adjustment
    );
    
    const float instant_emf_angle_variance = square(emf_axis_error);
    
    // Measure the noise of the angle error. We can't rely on the measured error above the configured noise threshold.
    const float emf_angle_variance = (
        readout.emf_angle_variance +
        (instant_emf_angle_variance - readout.emf_angle_variance) * control_parameters.emf_angle_ki
    );

    const float emf_voltage_magnitude = (
        readout.emf_voltage_magnitude + 
        (measured_emf_voltage_magnitude - readout.emf_voltage_magnitude) * control_parameters.emf_magnitude_ki
    );
    
    const float max_angle_error_variance = max(instant_emf_angle_variance, emf_angle_variance);
    
    // Check if the EMF angle is relatively stable. This is a proxy for detecting emf because at 0 speed, 0 emf, and
    // random noise readings for the u, v, w phases we should detect a random emf voltage of very low magnitude. Which
    // causes the angle to jump wildy and stabilize at about 90degree sqrt(variance).
    const bool emf_detected = max_angle_error_variance < emf_angle_variance_threshold;
    
    // Distance travelled by the emf axis on this timestep, directly converted to angular speed given the unit timestep.
    const float instant_emf_speed = emf_angle_adjustment + readout.angular_speed;

    const float emf_speed_error = instant_emf_speed - readout.emf_voltage_angular_speed;

    // Decay the emf speed to 0 if we don't have an emf detection.
    const float emf_voltage_angular_speed = (
        readout.emf_voltage_angular_speed + 
        (emf_detected ? emf_speed_error : -readout.emf_voltage_angular_speed) * control_parameters.emf_angular_speed_ki
    );

    const float instant_emf_speed_variance = square(emf_speed_error);

    const float emf_angular_speed_variance = (
        readout.emf_angular_speed_variance + 
        (instant_emf_speed_variance - readout.emf_angular_speed_variance) * control_parameters.emf_angular_speed_ki
    );

    const float max_emf_speed_variance = max(instant_emf_speed_variance, emf_angular_speed_variance);

    // Declare that we have an EMF fix if our speed is greater than the measurment error.
    const bool emf_fix = square(emf_voltage_angular_speed) > 4 * max_emf_speed_variance;
    
    // Track how many times we think our rotor angle is correct. Note that we keep the angle fix whilst the motor is off.
    correct_angle_counter = clip_to(
        0, control_parameters.angle_fix_max_certainty,
        // Subtract 1 for incorrect angles; otherwise add 1 for emf or hall angle fixes.
        // Our angle is incorrect if we don't have an EMF reading whilst driving the motor.
        correct_angle_counter + ((drive_state.pwm_value and not emf_detected) ? -1 : emf_detected)
    );
    
    // Declare the angle to be correct after a threshold certainty.
    // TODO: gotta fix this, it ain't doing too good
    const bool angle_fix = correct_angle_counter >= control_parameters.angle_fix_threshold_count;
    
    // Angle update
    // ------------
    
    // Integrate the EMF position error only if we're detecting EMF.
    const int32_t prediction_error = emf_detected * (emf_fix ? 
        // Aim to the axis corrected angle if we have certainty in the emf speed sign.
        (emf_voltage_angle + (emf_voltage_angular_speed > 0 ? quarter_circle : neg_quarter_circle) - predicted_angle) :
        // Otherwise adjust only towards the axis of the emf angle observer.
        (((emf_voltage_angle - predicted_angle) & most_positive_angle) - quarter_circle)
    );
    
    // Add the external angle offset to the angle adjustment.
    const int32_t angle_adjustment = prediction_error * control_parameters.rotor_angle_ki + external_angle_offset;
    external_angle_offset = 0;

    // Calculate the new angle based on the angle adjustment.
    const int32_t angle = predicted_angle + angle_adjustment;

    // Get the total angle change for the current cycle including adjustment and speed.
    const int32_t angle_diff = angle - readout.angle;

    // Check if the angle overflowed and count rotations. Note, we need to flag the compiler to treat
    // integer overflow as well defined behaviour!
    const int32_t rotations_increment = (angle_diff > 0 ? 
        (angle < readout.angle ? +1 : 0) :
        (angle > readout.angle ? -1 : 0)
    );

    // Calculate the new rotation index.
    const int32_t rotations = readout.rotations + rotations_increment + external_rotations_offset;
    external_rotations_offset = 0;
    
    // Calculate speed and acceleration
    // --------------------------------

    // Calculate the rotor speed as a low pass of the detected emf speed.
    const float angular_speed_error = readout.emf_voltage_angular_speed - readout.angular_speed;
    
    // Use the integral gain, but decay to 0 on loss of EMF certainty.
    const float angular_speed = (
        readout.angular_speed + 
        (emf_detected ? angular_speed_error : -readout.angular_speed) * control_parameters.rotor_angular_speed_ki
    );
    
    

    // Variable Tracking Update
    // ------------------------

    // Now, it's a bit of a cheat, but we'll use the new emf_voltage angle because we've computed whether to flip it
    // to minimize the error. The new angle and magnitude are already partially updated with the error so if we use
    // these we'll be left with less error by construction. Still the ki terms for the emf should be small and therefore
    // the difference is minimal. It is important that we use the flipped angle though.

    // Get the sin and cos for the emf angle.
    const float cos_emf = get_cos(emf_voltage_angle);
    const float sin_emf = get_sin(emf_voltage_angle);

    // Compute the predicted EMF voltage based on the running estimate at constant emf.
    const float direct_emf_voltage = emf_voltage_magnitude * cos_emf;
    const float quadrature_emf_voltage = emf_voltage_magnitude * sin_emf;


    // Now we track the resistance and inductance using some form of gradient descent. The equation that we want to solve is:
    // 
    //     0 ~ voltage_error = resistance_voltage + inductor_voltage - drive_voltage - emf_voltage
    // 
    // Where we've already defined the "measured" emf voltage as if the error were zero. Ie.
    // 
    //     measured_emf_voltage = resistance_voltage + inductor_voltage - drive_voltage
    // 
    // The error can also be due to either resistance or inductance, so despite naming it measured emf voltage, it's actually 
    // all the errors combined. We can use this value and calibrate all parameters live by gradient descent.
    const float direct_voltage_error = measured_direct_emf_voltage - direct_emf_voltage;
    const float quadrature_voltage_error = measured_quadrature_emf_voltage - quadrature_emf_voltage;

    // Following the formulas above, but this time we will try to estimate the inductor voltage given our other
    // masurements.
    const float direct_inductor_excitation = direct_inductor_voltage - direct_voltage_error;
    const float quadrature_inductor_excitation = quadrature_inductor_voltage - quadrature_voltage_error;

    const float square_voltage_error = square(direct_voltage_error) + square(quadrature_voltage_error);

    // Defining the loss as the square(direct_voltage_error) + square(quadrature_voltage_error) we get the derivative gradients with 
    // respect to each of the parameters we're tracking. We only care about the sign so we should be careful to skip updates when 
    // there isn't enough energy in the error to warrant an update.

    const bool error_detected = square_voltage_error > control_parameters.voltage_measurement_variance;

    // Run calibration when there's error or back EMF is detected.
    if (error_detected) {

        // Compute the gradients of the loss with respect to each of the tracked parameters.

        // Only update the resistance if we have sufficient current to make the measurement meaningful.
        if(current_magnitude > control_parameters.resistance_current_minimum) {
            const float resistance_gradient = (
                direct_voltage_error * direct_current +
                quadrature_voltage_error * quadrature_current
            ) * voltage_mul_current_to_power;

            current_calibration.resistance -= resistance_gradient * control_parameters.resistance_ki;
        }

        // Get the magnitude of the inductor driving voltage so we can determine whether the inductance is significant.
        const float inductor_excitation_square = square(direct_inductor_excitation) + square(quadrature_inductor_excitation);

        // Only update the inductance if the inductor is sufficiently excited to make up for the measurement noise.
        // Note the inductor noise is at least twice as much as the current noise because it is the difference of 2 measurements.
        if (inductor_excitation_square > control_parameters.inductance_excitation_minimum_square) {
            const float inductance_gradient = (
                direct_voltage_error * direct_inductance_base_voltage + 
                quadrature_voltage_error * quadrature_inductance_base_voltage
            ) * volts_per_voltage_units_square;

            current_calibration.inductance -= inductance_gradient * control_parameters.inductance_ki;

            const float inductance_bias_gradient = (
                direct_voltage_error * direct_inductance_bias_voltage +
                quadrature_voltage_error * quadrature_inductance_bias_voltage
            ) * volts_per_voltage_units_square;
            
            current_calibration.inductance_bias -= inductance_bias_gradient * control_parameters.inductance_bias_ki;

            const float saliency_angle_gradient = (
                direct_voltage_error * (-quadrature_inductance_bias_voltage) +
                quadrature_voltage_error * direct_inductance_bias_voltage
            ) * volts_per_voltage_units_square;

            current_calibration.saliency_angle_offset -= static_cast<int32_t>(saliency_angle_gradient * control_parameters.saliency_angle_ki);
        }
    } else {
        // Only track the zero offset while there is no noticeable current being measured.
        if (current_magnitude < control_parameters.current_offset_maximum) {
            // Finally if there's nothing unusual going on, we can update the zero offset for the currents.
            current_calibration.u_current_zero = clip_to(
                -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
                current_calibration.u_current_zero + std::get<0>(currents) * control_parameters.zero_current_ki);
            current_calibration.v_current_zero = clip_to(
                -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
                current_calibration.v_current_zero + std::get<1>(currents) * control_parameters.zero_current_ki);
            current_calibration.w_current_zero = clip_to(
                -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
                current_calibration.w_current_zero + std::get<2>(currents) * control_parameters.zero_current_ki);
        }
    }

    // Calculate the power use
    // -----------------------

    // Resistive power is the power dissipated in the motor coils and MOSFETs.
    const float resistive_power = (
        direct_resistive_voltage * direct_current + 
        quadrature_resistive_voltage * quadrature_current
    ) * dq0_voltage_mul_current_to_power;

    // EMF power is the power transferred into the rotor movement, driving the motor.
    const float emf_power = -(
        emf_voltage_magnitude * 
        current_magnitude * 
        get_cos(emf_voltage_angle - current_angle)
    ) * dq0_voltage_mul_current_to_power;

    // The total power is the power used from the battery. It will be positive when driving
    // the motor, meaning that we drain the battery. If this is negative it means we are charging
    // the battery by slowing down the motor (regenerative breaking).
    // 
    // The balance of all powers must be zero assuming no other source or sink of power. Thus
    // we can compute the total power from the others; mostly determined by EMF. The resistive
    // power is quite reliable and inductive_power is usually small (and sums to 0) but is 
    // very noisy so we will omit it.
    const float total_power = resistive_power + emf_power;


    // Limits!
    // -------

    // Calculate slowly varying averages of the resistive power; this represents the energy
    // dissipated in the motor coils which should be proportional to the temperature rise.
    // Update the higher resolution observer.
    const float resistive_power_average = (
        readout.resistive_power_average + 
        (resistive_power - readout.resistive_power_average) * control_parameters.resistive_power_ki
    );

    // Calculate slowly varying averages of the total power; this represents the energy
    // drawn from the battery. At constant voltage, this is proportional to the current drawn.
    const float total_power_average = (
        readout.total_power_average + 
        (total_power - readout.total_power_average) * control_parameters.power_draw_ki
    );


    // Write the latest readout data
    // -----------------------------
    // 
    // Must update the whole state before motor pwm calculation!

    readout.readout_number = readout_number;
    
    readout.angle_fix = angle_fix;
    
    readout.state_flags = (
        (hall_state << hall_state_bit_offset) |
        (current_detected << current_detected_bit_offset) |
        (emf_fix << emf_fix_bit_offset) |
        (emf_detected << emf_detected_bit_offset)
    );

    readout.direct_drive_voltage = direct_drive_voltage;
    readout.quadrature_drive_voltage = quadrature_drive_voltage;

    readout.direct_current = direct_current;
    readout.quadrature_current = quadrature_current;
    readout.zero_current = zero_current;

    readout.ref_readout = adc_readings.ref_readout;
    
    readout.direct_current_diff = direct_current_diff;
    readout.quadrature_current_diff = quadrature_current_diff;
    
    readout.angle = angle;
    readout.predicted_angle = predicted_angle;
    readout.angular_speed = angular_speed;
    readout.rotations = rotations;
    
    readout.current_angle_offset = current_angle_offset;
    readout.current_magnitude = current_magnitude;
    readout.emf_voltage_angle = emf_voltage_angle;
    readout.emf_voltage_magnitude = emf_voltage_magnitude;
    readout.emf_voltage_angular_speed = emf_voltage_angular_speed;
    readout.emf_angle_variance = emf_angle_variance;
    readout.emf_angular_speed_variance = emf_angular_speed_variance;

    readout.vcc_voltage = vcc_voltage;
    readout.temperature = temperature;
    readout.total_power = total_power;
    readout.total_power_average = total_power_average;
    readout.resistive_power_average = resistive_power_average;

    // Calculate and set motor outputs!!
    // ---------------------------------

    // Instantly set to 0, but otherwise approach the target PWM gradually.
    readout.active_pwm = drive_state.pwm_value == 0 ? 0 : (readout.active_pwm + clip_to(
        -control_parameters.max_pwm_change,
        +control_parameters.max_pwm_change,
        drive_state.pwm_value - readout.active_pwm
    ));

    // The main control flag is whether we drive in fixed coordinates (open loop) or relative coordinates (closed loop).
    // 
    // Breaking mode can be expressed as driving at 0 pwm in fixed coordinates.
    if (drive_state.enable_closed_loop) {
        // Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
        // close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
        // The PWM duty cycle, drive angle, and angle speed are varied smoothly.
        // Base the direction on the sign of the target PWM.
        const int32_t active_pwm_sign = sign(readout.active_pwm);

        // Drive towards the ideal angle; however decay to 0 at low EMF voltage.
        // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
        const int32_t ideal_angle = readout.angle + quarter_circle;

        // Drive the motor to produce current perpendicular to the magnetic angle.
        drive_state.angle = ideal_angle + active_pwm_sign * current_calibration.lead_angle;

    } else {
        // Drive the inductors around a circle at the specified PWM and speed (open loop control).
        drive_state.angle += static_cast<int32_t>(drive_state.angular_speed);
    }

    motor_outputs = compute_motor_outputs(
        drive_state.angle, 
        readout.active_pwm,
        current_calibration.live_max_pwm
    );

    // Disable the update for the control registers so we can write all 3.
    LL_TIM_DisableUpdateEvent(TIM1);

    // Send the command to the timer compare registers. Set the registers close to when cycle_end_tick 
    // is set so we can properly track the value for the next cycle. There's a half cycle delay if
    // we set the output registers too late in the cycle.
    // 
    // We also need to compensate for the turn on delay on each motor phase.
    // Note that we mustn't add the floor compensation to our calculation; it represents the 0 point for the PWM duty cycle.
    set_motor_outputs(readout.active_pwm == 0 ? breaking_motor_outputs : MotorOutputs{
        motor_outputs.u_duty + control_parameters.u_pwm_floor,
        motor_outputs.v_duty + control_parameters.v_pwm_floor,
        motor_outputs.w_duty + control_parameters.w_pwm_floor
    });
    
    // Re-enable the update for the control registers now that we've written all 3.
    LL_TIM_EnableUpdateEvent(TIM1);


    // End of cycle
    // ------------

    // Get the tick after we've written the motor control, we need to make sure this one is 
    // within the half cycle before the pwm registers update.
    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    // Write the latest readout to the history buffer for the main loop to read.
    readout_history_queue.write_if_available(readout);

    // Setup the new state if we were commanded by the main loop so we are prepared for the next cycle.
    // 
    // There are a few checks when copying to state, so to keep things glitchlessly fast we update it
    // after we've set the motor outputs. The new state will be ready for the next cycle.
    // 
    // Note, the new pending state may clear the readout history so this must be done after the history push.
    const auto pending_drive_states = drive_state_queue.read_available();


    if (pending_drive_states > drive_states_scheduled) {
        const size_t new_drive_states = pending_drive_states - drive_states_scheduled;
        // Skip these driver states as we've received new commands; always prioritize the latest batch.
        drive_state_queue.mark_read(drive_states_scheduled);
        // Set the new schedule count and load the first driver state.
        drive_states_scheduled = new_drive_states;

        // Copy the pending driver state.
        // 
        // ! Warning, we rely on the main loop to clamp the commands to valid ranges!
        drive_state = drive_state_queue.peek();

        // Immediately set the motor flags; once per new drive state.
        set_motor_status(drive_state.motor_flags);
    } else if (drive_state.duration == 0) {
        // If we were scheduling a bunch of driver states, decrement the scheduled count and mark executed
        // state as read.
        if (drive_states_scheduled >= 2) {
            drive_states_scheduled--;
            drive_state_queue.mark_read(1);
            drive_state = drive_state_queue.peek();
        } else if (drive_states_scheduled == 1) {
            // We were at the last scheduled driver state; so now we're done and we transition to breaking.
            drive_states_scheduled = 0;
            // Set the driver state to the safe breaking state if the duration has expired.
            drive_state = breaking_drive_state;
            // Mark this as read, but do we have nothing else to peek at.
            drive_state_queue.mark_read(1);
        }
        // Immediately set the motor flags; once per new drive state.
        set_motor_status(drive_state.motor_flags);
    } else {
        // Count down the active duration.
        drive_state.duration--;
    }

    // TODO: we need complete nan checks; preferably in the main loop.
    if (std::isnan(readout.emf_voltage_magnitude)) {
        readout.emf_voltage_magnitude = 0.f;
    }

    // Clear the ADC end of conversion flag so we're ready for the next conversion.
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
}