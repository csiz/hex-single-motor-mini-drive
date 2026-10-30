#include "interrupts.hpp"

#include "hex_mini_drive_interface.hpp"

#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "io.hpp"
#include "constants.hpp"
#include "dq0_transform.hpp"
#include "circular_buffer.hpp"

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
// 
// For our sanity we also want to keep the loop state condensed into as few variables as possible.

// Electrical and position state
hex_mini_drive::Readout readout = {};


// Additional state
// ----------------
// 
// Unfortunately we need a tiny bit more state than the one we can serialize over the wire.
// Maybe there's a way around it, but the big problem is we have to average motor outputs
// over two PWM cycles to get the actual output.

// Settings for the 3 phases (on or tri-state).
uint8_t motor_flags = 0;

// Settings for the 3 phase MOSFET PWM drivers.
MotorOutputs motor_outputs = breaking_motor_outputs;

// Our outputs are delayed 1 cycle; store the previous outputs here before we use them.
MotorOutputs previous_motor_outputs = breaking_motor_outputs;

// Angle adjustment from the main loop.
volatile int32_t external_angle_adjustment = 0;

// Average out temperature reading for the main loop to calculate.
float temperature_reading = 0.f;

// Default drive state to use when no command is available.
hex_mini_drive::DriveState default_drive_state = breaking_drive_state;


// Interrupt Data Interface
// ------------------------

// Initialize the loop control parameters and the calibration data. Either load 
// them from the flash or use the defaults. Docstrings are in the header file.

hex_mini_drive::Info info = {/* all default to 0 */};

hex_mini_drive::Calibration calibration = get_calibration();

hex_mini_drive::ControlParameters control_parameters = get_control_parameters();

CircularQueue<hex_mini_drive::Readout, hex_mini_drive::HISTORY_SIZE> readout_history_queue = {};

CircularQueue<hex_mini_drive::DriveState, hex_mini_drive::DRIVE_SCHEDULE_SIZE + 1> drive_state_queue = {};

bool is_motor_safed(){
    // Consider both motor breaking and freewheeling as safe states.
    return (readout.active_pwm == 0.f);
}

bool set_angle_adjustment(int32_t const& adjustment){
    if (external_angle_adjustment) return false;
    external_angle_adjustment = adjustment;
    return true;
}

float get_temperature(){
    return temperature_reading;
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

// Get the 3 phase motor outputs to drive a voltage vector specified by angle and pwm magnitude.
static inline MotorOutputs compute_motor_outputs(int32_t const& angle, float const& pwm_value) {

    // Get the voltage for the three phases from the waveform table.
    const float voltage_phase_u = get_phase_pwm(angle);
    const float voltage_phase_v = get_phase_pwm(angle - third_circle);
    const float voltage_phase_w = get_phase_pwm(angle - neg_third_circle);

    return MotorOutputs{
        .u_duty = static_cast<uint16_t>(voltage_phase_u * pwm_value),
        .v_duty = static_cast<uint16_t>(voltage_phase_v * pwm_value),
        .w_duty = static_cast<uint16_t>(voltage_phase_w * pwm_value)
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
        -static_cast<float>(adc_readings.u_readout - adc_readings.ref_readout) * adc_to_current_units - calibration.u_current_zero,
        -static_cast<float>(adc_readings.v_readout - adc_readings.ref_readout) * adc_to_current_units - calibration.v_current_zero,
        -static_cast<float>(adc_readings.w_readout - adc_readings.ref_readout) * adc_to_current_units - calibration.w_current_zero
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

    // Switching to DQ0 Frame
    // ----------------------
    // 
    // Calculate the park transformed currents and voltages: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
    // 
    // We reduce the number of operations and allow for more uses of constants if we 
    // immediately transform to the dq0 frame at the fixed angle 0. We can always
    // rotate the frame afterwards using a 2D rotation matrix with just 4 elements.


    // And transform the drive voltages too.
    const auto [direct_drive_voltage, quadrature_drive_voltage] = dq0_transform(drive_voltages);

    // Cast the angle speed to angle units per pwm cycle.
    const int32_t angle_traversed = readout.angular_speed;

    // Predict the position; keeping track of fractional angles at the same resolution as
    // the speed. By our definition the time unit is 1 per cycle; so the angle spanned by 
    // the rotor is exactly the angular speed.
    const int32_t predicted_angle = readout.angle + angle_traversed;


    // Current angle calculation
    // -------------------------
    // 
    // We calculate the angle of the current vector that is running through the motor coils.
    // 
    // In our convention the inductors driven with positive current form a south pole that attracts
    // the north pole of the rotor.

    // Transform the measured currents into the DQ0 frame at the fixed angle 0.
    const auto [direct_current, quadrature_current] = dq0_transform(currents);
    
    // Invoke the CORDIC engine to compute atan2 and magnitude using the phase function.
    set_cordic(direct_current, quadrature_current);

    // Intermingle other calculations while the cordic calculates.

    // Get the common mode current. It should be 0 in theory, but of course it is not in practice...
    const float zero_current = (std::get<0>(currents) + std::get<1>(currents) + std::get<2>(currents)) * one_third;

    // Calculate the angle at which the current is running on the motor coils, averaged the same way as we do for emf.
    const auto [current_angle, current_magnitude] = get_cordic();

    const int32_t current_angle_offset = current_angle - predicted_angle;


    // Using DQ0 coordinates
    // ---------------------
        
    // Calculate the predicted EMF angle.
    const int32_t predicted_emf_voltage_angle = readout.emf_voltage_angle + angle_traversed;

    const float cos_emf = get_cos(predicted_emf_voltage_angle);
    const float sin_emf = get_sin(predicted_emf_voltage_angle);

    const float predicted_direct_emf_voltage = readout.emf_voltage_magnitude * cos_emf;
    const float predicted_quadrature_emf_voltage = readout.emf_voltage_magnitude * sin_emf;

    // Calculate the resistive voltage drop across the coil and MOSFET resistance.
    const float direct_resistive_voltage = direct_current * calibration.resistance;
    const float quadrature_resistive_voltage = quadrature_current * calibration.resistance;

    // Calculate the differential of the currents; this will be the real measurement of our model's prediction.
    const float direct_current_diff = direct_current - readout.direct_current;
    const float quadrature_current_diff = quadrature_current - readout.quadrature_current;

    const float direct_diff = direct_current_diff * current_diff_to_voltage_units;
    const float quadrature_diff = quadrature_current_diff * current_diff_to_voltage_units;

    // Ignore the saliency_angle_offset in the tight loop and assume it is 0. The saliency
    // effect depends on the rotor position!
    const int32_t saliency_angle = 2*predicted_angle;

    const float cos_saliency = get_cos(saliency_angle);
    const float sin_saliency = get_sin(saliency_angle);

    const float direct_inductance_bias_voltage = calibration.inductance_bias * (
        -direct_diff * cos_saliency 
        -quadrature_diff * sin_saliency
    );
    const float quadrature_inductance_bias_voltage = calibration.inductance_bias * (
        -direct_diff * sin_saliency
        +quadrature_diff * cos_saliency
    );

    const float direct_inductance_base_voltage = direct_diff * calibration.inductance;
    const float quadrature_inductance_base_voltage = quadrature_diff * calibration.inductance;

    const float direct_inductor_voltage = direct_inductance_base_voltage + direct_inductance_bias_voltage;
    const float quadrature_inductor_voltage = quadrature_inductance_base_voltage + quadrature_inductance_bias_voltage;

    const float measured_direct_emf_voltage = direct_inductor_voltage + direct_resistive_voltage - direct_drive_voltage;
    const float measured_quadrature_emf_voltage = quadrature_inductor_voltage + quadrature_resistive_voltage - quadrature_drive_voltage;

    const float direct_emf_voltage_error = measured_direct_emf_voltage - predicted_direct_emf_voltage;
    const float quadrature_emf_voltage_error = measured_quadrature_emf_voltage - predicted_quadrature_emf_voltage;

    const float direct_emf_voltage = predicted_direct_emf_voltage + direct_emf_voltage_error * control_parameters.loop_fast_ki;
    const float quadrature_emf_voltage = predicted_quadrature_emf_voltage + quadrature_emf_voltage_error * control_parameters.loop_fast_ki;

    // Send the emf values to the CORDIC asap.
    set_cordic(direct_emf_voltage, quadrature_emf_voltage);

    // More unrelated calculations intermingled.

    // Average the temperature readings since we are sampling quicker than the manufacturer indicates.
    // We can't extend the sampling time longer than it is set at the moment (about half the recommendation),
    // so we have to massage the readings for noise. Temperature varies slowly anyway.
    temperature_reading = (adc_readings.temp_readout * 0.25f + temperature_reading * 0.75f);

    // Back EMF angle observer
    // -----------------------

    // Get the angle measured from EMF relative to the predicted rotor angle.
    const auto [emf_voltage_angle, emf_voltage_magnitude] = get_cordic();

    // Distance travelled by the emf axis on this timestep, directly converted to angular speed given our unit timestep convention.
    const float emf_speed_error = (emf_voltage_angle - readout.emf_voltage_angle) - readout.emf_voltage_angular_speed;
    
    // Note that the sliding mode emf observer is based on the rotor angular_speed, so we are free to update the emf angular speed
    // estimate independently. We update it all the time so we can calculate the baseline variance for the angular speed.
    const float emf_voltage_angular_speed = readout.emf_voltage_angular_speed + emf_speed_error * control_parameters.loop_fast_ki;
    
    // For the emf voltage variance we add the errors in the frame before rotating.
    const float instant_emf_voltage_variance = (
        square(direct_emf_voltage_error) +
        square(quadrature_emf_voltage_error)
    );

    // Use the worst of the instantaneous variance or the filtered variance to track the worst case variance.
    const float emf_magnitude_variance = (
        readout.emf_magnitude_variance +
        (instant_emf_voltage_variance - readout.emf_magnitude_variance) * control_parameters.loop_fast_ki
    );


    // Compute a second tracker for the emf magnitude that is lagged exactly the same as the angular speed estimate
    // so that we can compute an unbiased motor constant.
    const float emf_stable_magnitude = readout.emf_stable_magnitude + (emf_voltage_magnitude - readout.emf_stable_magnitude) * control_parameters.loop_fast_ki;

    // Check if the EMF angle is relatively stable. This is a proxy for detecting emf because at 0 speed, 0 emf, and
    // random noise readings for the u, v, w phases we should detect a random emf voltage of very low magnitude. Which
    // causes the angle to jump wildy and stabilize at about 90degree sqrt(variance).
    const bool emf_detected = square(emf_voltage_magnitude) > emf_magnitude_variance;

    // The variance will be the square of the angle error, it should fluctuate wildly at standstill when the emf vector is near 0.
    const float instant_emf_angle_variance = square(emf_voltage_angle - predicted_emf_voltage_angle);

    // Track the worst case variance for speed, the max of the instantaneous speed variance and the filtered speed variance.
    const float emf_angle_variance = (
        readout.emf_angle_variance + 
        (instant_emf_angle_variance - readout.emf_angle_variance) * control_parameters.loop_fast_ki
    );

    // Calculate the instant motor constant
    // ------------------------------------
    // 
    // Instantaneous motor constant; ratio of EMF voltage magnitude to emf angular speed; 
    // gives an estimate of the motor's back-EMF constant in the interrupt control loop.
    // We use this faster estimate to deduce the real rotor angular speed because it looks
    // like it depends much more reliably on the EMF magnitude rather than the EMF angle
    // estimation. Because the EMF response is quite noisy around motor parameter shifts
    // the EMF angle is often distorted by a noise of comparable magnitude to the EMF signal.
    // When differentiated, this noise often oscillates to negative values, while the emf
    // magnitude changes much more slowly despite being optimized with the same learning rate.

    const float predicted_emf_speed_constant = emf_detected ? 
        ((emf_voltage_angular_speed * emf_speed_constant_conversion_inverse) / emf_stable_magnitude) :
        0.0f;

    // Get the error (gradient) for the motor constant observer.
    const float emf_speed_constant_error = (predicted_emf_speed_constant - readout.emf_speed_constant);

    const float emf_speed_constant = readout.emf_speed_constant + control_parameters.emf_speed_constant_ki * emf_speed_constant_error;

    const float instant_emf_speed_constant_variance = square(emf_speed_constant_error) + min_speed_constant_sq;

    const float emf_speed_constant_variance = (
        readout.emf_speed_constant_variance + 
        (instant_emf_speed_constant_variance - readout.emf_speed_constant_variance) * control_parameters.emf_speed_constant_ki
    );

    // Calculate motor constant, speed, and acceleration
    // -------------------------------------------------
    
    const bool emf_angle_fix = emf_detected and (emf_angle_variance < control_parameters.emf_angle_suspected_threshold);

    const float emf_speed_constant_sq = square(emf_speed_constant);

    // Calculate the rotor speed as a low pass of the detected emf speed.
    const float angular_speed_error = emf_detected * (emf_speed_constant * emf_speed_constant_conversion * emf_voltage_magnitude) - readout.angular_speed;

    // Use the integral gain, but decay to 0 on loss of EMF certainty.
    const float angular_speed = readout.angular_speed + angular_speed_error * control_parameters.angle_from_emf_ki;

    // Declare that we have an EMF fix if our speed is greater than the measurment error.
    const bool emf_fix = emf_speed_constant_sq > emf_speed_constant_variance * control_parameters.emf_speed_constant_confirmed_threshold;


    // Angle update
    // ------------
    
    // Integrate the EMF position error only if we're detecting EMF.
    const int32_t angle_adjustment = emf_angle_fix * control_parameters.angle_from_emf_ki * (emf_fix ? 
        // Aim to the axis corrected angle if we have certainty in the emf speed sign.
        (emf_voltage_angle + (angular_speed > 0 ? quarter_circle : neg_quarter_circle) - predicted_angle) :
        // Otherwise adjust only towards the axis of the emf angle observer, with an extra downscaling.
        (((emf_voltage_angle - predicted_angle) & most_positive_angle) - quarter_circle) * control_parameters.angle_from_emf_ki
    );
    
    // Calculate the new angle based on all the angle adjustments.
    const int32_t angle = predicted_angle + angle_adjustment + external_angle_adjustment;

    // Clear the external angle adjustment after applying it.
    external_angle_adjustment = 0;

    // Get the total angle change for the current cycle including adjustment and speed.
    const int32_t angle_diff = angle - readout.angle;

    // Check if the angle overflowed and count rotations. Note, we need to flag the compiler to treat
    // integer overflow as well defined behaviour!
    const int32_t rotations_increment = (angle_diff > 0 ? 
        (angle < readout.angle ? +1 : 0) :
        (angle > readout.angle ? -1 : 0)
    );

    // Calculate the new rotation index.
    const int32_t rotations = readout.rotations + rotations_increment;
  

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

    // Safety Limits
    // -------------
    // Reduce the maximum output PWM to keep within safe limits:
    // 1. The MOSFET drivers need to be kept in their operating voltage range. Reduce PWM to
    // let the battery recharge our local capacitors.
    // 2. The resistive power heats up the motor coils. Keep it under a threshold to avoid overheating.
    // 3. The total power is a good proxy for total current consumed from the battery.
    // 
    // The penalty should normally be negative indicating we can increase the PWM.
    const bool pwm_too_high = (
        (vcc_voltage < control_parameters.vcc_undervoltage) or
        (resistive_power_average > control_parameters.max_resistive_power) or
        (total_power_average > control_parameters.max_power_draw)
    );

    const float pwm_penalty = (0.1f - 1.f * pwm_too_high);

    const float live_max_pwm = clip_to(0, pwm_max, readout.live_max_pwm + pwm_penalty);


    // Calculate and set motor outputs!!
    // ---------------------------------


    // Check if there is a command from the main loop.
    const bool drive_state_available = drive_state_queue.read_available();

    auto & drive_state = drive_state_available ? drive_state_queue.modify() : default_drive_state;

    // Drive in closed loop control relative to the rotor angle and targetting a max torque and speed.
    if (drive_state.closed_loop) {
        // Get the rotation direction from the target speed.
        const int32_t target_sign = (drive_state.target_speed < 0) ? -1 : 1;
        // Use the previous sign to determine the absolute value of the target speed.
        const float target_abs_speed = drive_state.target_speed * target_sign;
        // Adjust the actual speed based on the target direction (it's cheaper to reuse
        // the target_sign this way than to compute the absolute speed, it also
        // gives us the correct distance in speed units betwen target and actual speed).
        const float sign_adjusted_speed = angular_speed * target_sign;
        // Use the sign of the target speed to determine the direction of the angle offset.
        const int32_t target_angle_offset = drive_state.angle_value * target_sign;
            
        // Get the pwm adjustment to obtain the desired current magnitude.
        const float current_adjustment = (
            (drive_state.target_value - current_magnitude) * 
            control_parameters.torque_control_ki
        );

        // Get the speed adjustment to obtain the desired speed.
        const float speed_adjustment = (
            (target_abs_speed - sign_adjusted_speed) * 
            control_parameters.speed_control_ki
        );

        // Use the conservative PWM to achieve either the target speed or commanded current magnitude, whichever comes first.
        readout.active_pwm = clip_to(0, live_max_pwm, readout.active_pwm + min(speed_adjustment, current_adjustment));
        
        // Target the sign adjusted angle_value normally.
        readout.active_angle = readout.active_angle + static_cast<int32_t>(angular_speed + (
            (target_angle_offset - current_angle_offset) * control_parameters.lead_angle_control_ki
        ));
        
        
        // TODO: fix the breaking logic or maybe make it a special command from the main loop.
        // Hmm, maybe we can use the driver_state.angle_value being opposite of angular_speed
        // as a signal that we want to achieve breaking.
        
        // // We're breaking if the motor is moving the opposite direction the target speed.
        // const bool breaking = emf_fix and sign_adjusted_speed < 0;
        
        // // We need to treat breaking case separately because all behaviour reverses sign
        // // when we try to absorb energy from the rotor (regenerative braking).
        // if (breaking) {
        //     // If breaking we need to reverse our pwm and angle adjustments to max out on the
        //     // commanded current magnitude (regenerative breaking). Because to reduce current
        //     // absorbed during breaking, we need to increase PWM! So that we compensate for
        //     // the back EMF harder.
        //     active_pwm = clip_to(0, live_max_pwm, active_pwm - current_adjustment);
        //     // Target the mirror current angle if breaking.
        //     active_angle += static_cast<int32_t>(angular_speed + (
        //         (target_angle_offset - current_angle_offset + half_circle) * 
        //         control_parameters.lead_angle_control_ki
        //     ));
        // }
    // Drive in open loop control.
    } else {
        // Drive the inductors around a circle at the specified PWM and speed. And apply the initial offset.
        readout.active_angle = readout.active_angle + static_cast<int32_t>(drive_state.target_speed) + drive_state.angle_value;
        // Zero out the offset so we don't apply it again in the next control cycle.
        drive_state.angle_value = 0;
        // Copy the commanded PWM value from the drive command to our readout state.
        readout.active_pwm = clip_to(0, live_max_pwm, drive_state.target_value);
    }

    if (drive_state_available) {
        // Count down execution time or discard the top drive state when we finish its duration.
        if (drive_state.duration) drive_state.duration -= 1;
        else drive_state_queue.mark_read(1);
    } else {
        // Set the active_angle to the rotor angle while there's no drive command.
        readout.active_angle = angle;
    }

    readout.probing_angle += drive_state.probing_speed_modulation;

    
    // Calculate and store desired motor pwm;
    motor_outputs = compute_motor_outputs(readout.active_angle, readout.active_pwm);

    // Send the command to the timer compare registers. Set the registers close to when cycle_end_tick 
    // is set so we can properly track the value for the next cycle. There's a half cycle delay if
    // we set the output registers too late in the cycle.
    // 
    // We also need to compensate for the turn on delay on each motor phase.
    // 
    // Note that we mustn't add the floor compensation to our calculation; it represents 
    // the 0 point for the PWM duty cycle. That's why we store the desired PWM setting in motor_outputs
    // but we add the floor compensation only when writing to the actual hardware registers.
    // 
    // Also note that we want to write all zeroes when the motor is actually breaking so we don't
    // drain extra power trying to half-switch the mosfets for a tiny amount of time.
    set_motor_outputs(MotorOutputs{
        motor_outputs.u_duty ? motor_outputs.u_duty + control_parameters.u_pwm_floor : 0,
        motor_outputs.v_duty ? motor_outputs.v_duty + control_parameters.v_pwm_floor : 0,
        motor_outputs.w_duty ? motor_outputs.w_duty + control_parameters.w_pwm_floor : 0
    });

    // Enable phases based on the motor_flags settings.
    set_motor_status(drive_state.motor_flags);


    
    // End of cycle
    // ------------

    // Get the tick after we've written the motor control, we need to make sure this one is 
    // within the half cycle before the pwm registers update.
    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    
    // TODO: we need complete nan checks; preferably in the main loop.
    if (std::isnan(readout.emf_voltage_magnitude)) {
        readout.emf_voltage_magnitude = 0.f;
    }

    // Write the latest readout data
    // -----------------------------

    // TODO: Copy the motor flags somewhere?
    // motor_flags = drive_state.motor_flags;

    readout.readout_number = readout_number;
    readout.ref_readout = adc_readings.ref_readout;
    
    readout.direct_drive_voltage = direct_drive_voltage;
    readout.quadrature_drive_voltage = quadrature_drive_voltage;

    readout.direct_current = direct_current;
    readout.quadrature_current = quadrature_current;
    readout.zero_current = zero_current;
    readout.direct_current_diff = direct_current_diff;
    readout.quadrature_current_diff = quadrature_current_diff;
    
    readout.rotations = rotations;
    readout.angle = angle;
    readout.predicted_angle = predicted_angle;
    readout.angular_speed = angular_speed;
    readout.acceleration = 0; // TODO

    readout.vcc_voltage = vcc_voltage;
    readout.emf_voltage_angle = emf_voltage_angle;
    readout.emf_voltage_magnitude = emf_voltage_magnitude;
    readout.emf_magnitude_variance = emf_magnitude_variance;
    readout.emf_voltage_angular_speed = emf_voltage_angular_speed;
    readout.emf_angle_variance = emf_angle_variance;

    readout.emf_speed_constant = emf_speed_constant;
    readout.emf_speed_constant_variance = emf_speed_constant_variance;

    readout.current_angle_offset = current_angle_offset;
    readout.current_magnitude = current_magnitude;

    readout.live_max_pwm = live_max_pwm;
    readout.total_power_average = total_power_average;
    readout.resistive_power_average = resistive_power_average;

    readout.emf_stable_magnitude = emf_stable_magnitude;

    // Write the latest readout to the history buffer for the main loop to read.
    readout_history_queue.write_if_available(readout);

    // Clear the ADC end of conversion flag so we're ready for the next conversion.
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC2);
}