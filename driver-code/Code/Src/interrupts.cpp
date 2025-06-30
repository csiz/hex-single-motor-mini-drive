#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_pid.hpp"
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
Observers observers = {};

// Count the number of consecutive EMF detections; we get good statistics after a threshold number.
size_t consecutive_emf_detections = 0;


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

// PID control state for all controls.
PIDControlState pid_state = null_pid_control_state;


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

    // Remember the previous speed for acceleration calculations.
    const int previous_angular_speed = readout.angular_speed;

    // Predict the position based on the previous state.
    const auto predicted_position = predict_position(observers.rotor_position);

    observers.rotor_position = predicted_position;

    // Get calibrated currents.

    const auto [u_current, v_current, w_current] = adjust_to_sum_zero(ThreePhase{
        u_readout * current_calibration.u_factor / current_calibration_fixed_point,
        v_readout * current_calibration.v_factor / current_calibration_fixed_point,
        w_readout * current_calibration.w_factor / current_calibration_fixed_point
    });

    // Get calibrated current divergence (the time unit is defined 1 per cycle).

    const auto [u_current_diff, v_current_diff, w_current_diff] = adjust_to_sum_zero(ThreePhase{
        u_readout_diff * current_calibration.u_factor / current_calibration_fixed_point,
        v_readout_diff * current_calibration.v_factor / current_calibration_fixed_point,
        w_readout_diff * current_calibration.w_factor / current_calibration_fixed_point
    });

    // Calibrated conversion factor between current divergence and phase inductance voltage.
    const int diff_to_voltage = round_div(phase_readout_diff_per_cycle_to_voltage * current_calibration.inductance_factor, current_calibration_fixed_point);

    // Calculate the voltage drop across the coil inductance.

    const int u_inductor_voltage = u_current_diff * diff_to_voltage / current_fixed_point;
    const int v_inductor_voltage = v_current_diff * diff_to_voltage / current_fixed_point;
    const int w_inductor_voltage = w_current_diff * diff_to_voltage / current_fixed_point;

    // Conversion factor between current and phase resistance voltage.
    const int phase_current_to_voltage = round_div(phase_resistance * voltage_fixed_point, resistance_fixed_point);

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

    const int u_emf_voltage = -(u_drive_voltage - u_resistive_voltage - u_inductor_voltage);
    const int v_emf_voltage = -(v_drive_voltage - v_resistive_voltage - v_inductor_voltage);
    const int w_emf_voltage = -(w_drive_voltage - w_resistive_voltage - w_inductor_voltage);

    // Load the projections of each phase on the current electric angle.

    const int16_t cos_u = get_cos(observers.rotor_position.angle);
    const int16_t cos_v = get_cos(observers.rotor_position.angle - third_circle);
    const int16_t cos_w = get_cos(observers.rotor_position.angle - two_thirds_circle);

    // We need the negative sine values for the park transformation.

    const int16_t neg_sin_u = -get_sin(observers.rotor_position.angle);
    const int16_t neg_sin_v = -get_sin(observers.rotor_position.angle - third_circle);
    const int16_t neg_sin_w = -get_sin(observers.rotor_position.angle - two_thirds_circle);

    // Calculate the park transformed currents.
    // 
    // The beta component contributes to torque. The alpha component should be driven to zero, but it
    // can also be used to hold the motor at standstill. When we stay within a single hall sector, the
    // angle estimate degrades to +-30 degrees. A holding current would exert a torque proportional to
    // the deviation between the real and target angle even when we can't sense the real angle.

    const int alpha_current = (
        u_current * cos_u / angle_base +
        v_current * cos_v / angle_base +
        w_current * cos_w / angle_base
    );

    const int unadjusted_beta_current = (
        u_current * neg_sin_u / angle_base +
        v_current * neg_sin_v / angle_base +
        w_current * neg_sin_w / angle_base
    );

    // Calculate the park transformed EMF voltages.
    // 
    // Use the kalman filtered angle estimates. These are updated on the order of 2KHz while the PWM
    // cycle is at 23KHz. The angle might diverge from reality and we can sense it by the misalignment
    // in the park transformed voltages. We should adjust our angle estimate such that our calculations
    // obtain an alpha emf voltage close to zero. The EMF voltage is a source of information about the
    // rotor position and speed. But we don't have compute to calculate the angle, so we treat it as an
    // optimization problem over multiple cycles.
    // 
    // Exponentially average the values below to reduce noise, 1 : 3 parts coorresponds to 150us half life
    // at our cycle frequency.

    const int alpha_emf_voltage = (
        u_emf_voltage * cos_u / angle_base +
        v_emf_voltage * cos_v / angle_base +
        w_emf_voltage * cos_w / angle_base
    );


    const int beta_emf_voltage = (
        u_emf_voltage * neg_sin_u / angle_base +
        v_emf_voltage * neg_sin_v / angle_base +
        w_emf_voltage * neg_sin_w / angle_base
    );


    // This might be a good proxy for the variance of the EMF voltage.
    const int emf_voltage_variance = clip_to(1, max_16bit, round_div(
        square(alpha_emf_voltage) +
        3 * readout.emf_voltage_variance,
        4
    ));

    // We need to compensate the EMF voltage for our position error. As long as we have an emf fix,
    // then any stray alpha voltage is from unaccounted rotor acceleration.
    const int emf_voltage = beta_emf_voltage;

    const int sq_emf_voltage = square(emf_voltage);

    // Check if the emf voltage is away from zero with enough confidence (2 standard deviations).
    consecutive_emf_detections = (sq_emf_voltage > 4 * emf_voltage_variance) * (consecutive_emf_detections + 1);

    const bool emf_detected = consecutive_emf_detections > 8;

    // The rotation direction is the sign of the negative of the beta EMF voltage.
    const bool emf_direction_is_negative = emf_voltage > 0;

    const int emf_direction = 1 - emf_direction_is_negative * 2;

    // The EMF voltage gives us a noisy but accurate estimate of the rotor magnetic angle;
    // use it to update the position between hall sensor toggles.
    if (consecutive_emf_detections > 3) {
        // Poor man's atan2, what we need to do is drive the alpha current to zero relative to 
        // the beta current. Since calculating the angle is expensive just consider the ratio.
        const int approximate_angle_error = -clip_to(
            -quarter_circle, +quarter_circle, 
            emf_detected * quarter_circle * alpha_emf_voltage / emf_voltage
        );

        const int emf_angle_variance = clip_to(1, max_16bit, emf_initial_angular_variance * emf_voltage_variance / sq_emf_voltage);

        observers.rotor_position = bayesian_update(
            observers.rotor_position,
            PositionStatistics{
                .angle = approximate_angle_error,
                .angle_variance = emf_angle_variance,
                .angular_speed = approximate_angle_error * speed_fixed_point,
                .angular_speed_variance = min(emf_angle_variance * speed_variance_fixed_point, max_16bit)
            }
        );
    }


    const int beta_current = unadjusted_beta_current + alpha_current * emf_direction / 2;


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
    // const int emf_power = -(
    //     u_current * u_emf_voltage + 
    //     v_current * v_emf_voltage + 
    //     w_current * w_emf_voltage
    // ) / voltage_current_div_power_fixed_point;

    // const int total_power = -(
    //     u_current * u_drive_voltage + 
    //     v_current * v_drive_voltage + 
    //     w_current * w_drive_voltage
    // ) / voltage_current_div_power_fixed_point;

    // By assuming the motor moves smoothly we claim the beta current is closer to the
    // measured magnitude of the DQ0 current. We then adjust our beta current using
    // a Taylor expansion for small alpha current. The system uses the real beta current
    // not what we measure; thus we compute the emf power using our massaged DQ0 values.
    const int emf_power = -beta_current * emf_voltage / dq0_to_power_fixed_point;

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

    // Get the external acceleration torque by subtracting our EMF acceleration from our position measurements.
    // 
    // Only add speed changes when we have an EMF fix; the speed will jump a large amount on the first position detection.
    const int rotor_acceleration = emf_detected * (observers.rotor_position.angular_speed - previous_angular_speed) * acceleration_fixed_point;

    const bool valid_motor_constant = (emf_detected and (abs(observers.rotor_position.angular_speed) > min_emf_angular_speed));

    const int instant_motor_constant = - valid_motor_constant * emf_voltage * motor_constant_fixed_point / observers.rotor_position.angular_speed;


    // Write the latest readout data
    // -----------------------------

    readout.pwm_commands = encode_pwm_commands(motor_outputs);

    // Advance the readout number.
    readout.readout_number = readout.readout_number + 1;
    
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

    // The angle is 10 bits so we have 6 bits left to encode the hall state, angle validity, emf detection (when the rotor moves), and direction.
    readout.angle = (
        (observers.rotor_position.angle & angle_bit_mask) | 
        (readout.angle & hall_state_bit_mask) | 
        (readout.angle & angle_valid_bit_mask) | 
        (emf_detected << emf_detected_bit_offset) | 
        (emf_direction_is_negative << emf_direction_is_negative_bit_offset)
    );
    readout.angle_variance = observers.rotor_position.angle_variance;
    readout.angular_speed = observers.rotor_position.angular_speed;
    readout.angular_speed_variance = observers.rotor_position.angular_speed_variance;

    readout.alpha_current = alpha_current;
    readout.beta_current = beta_current;

    readout.motor_constant = instant_motor_constant;
    readout.emf_voltage = emf_voltage;
    readout.emf_voltage_variance = emf_voltage_variance;
    readout.residual_acceleration = rotor_acceleration;

    readout.total_power = total_power;
    readout.resistive_power = resistive_power;
    readout.emf_power = emf_power;
    readout.inductive_power = inductive_power;


    // Calculate motor outputs and write control state
    // -----------------------------------------------

    // Update the motor controls using the readout data. Note that it also modifies the driver_state, driver_parameters, and pid_state.
    active_motor_outputs = update_motor_control(pending_state, pending_parameters, readout, driver_state, driver_parameters, pid_state);

    // Always reset the pending state to be ready for the next command.
    pending_state = DriverState::NO_CHANGE;


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


// Write only variable, for funsies.
volatile int write_only = 0;

void initialize_angle_tracking(){
    // Load the phase and sin tables into memory.
    for (int i = 0; i < angle_base; i++) {
        write_only = get_phase_pwm(i);
        write_only = get_sin(i);
    }
}
