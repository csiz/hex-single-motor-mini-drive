#include "interrupts.hpp"
#include "interrupts_data.hpp"
#include "interrupts_angle.hpp"
#include "interrupts_motor.hpp"

#include "parameters_store.hpp"

#include "io.hpp"
#include "constants.hpp"
#include "type_definitions.hpp"
#include "integer_math.hpp"

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

// Interrupt loop
// ==============

// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.


// Interrupt Loop State
// --------------------

// Electrical and position state
FullReadout readout = {
    .live_max_pwm = pwm_max,
    .emf_angle_error_variance = max_16bit,
};

// History of light readouts so that we can record every cycle for a short snapshot.
Readout readout_history[history_size] = {};

// Current write index.
size_t readout_history_write_index = 0;

// Current read index.
size_t readout_history_read_index = 0;


// Additional state
// ----------------

// Count the number of consecutive EMF detections.
int number_of_emf_detections = 0;

const int emf_speed_threshold = 16;
const int emf_fix_threshold = 32;
const int emf_fix_max = 64;

// Count for our belief that the rotor direction is incorrect. The EMF gives the axis
// of the rotor magnetic field, but we can have a 180 degree error in the direction
// until we identify the rotation direction.
int incorrect_direction_detections = 0;

const int incorrect_direction_warning = 64;

// Track how many times we think our angle is correct.
int correct_angle_counter = 0;

const int angle_fix_threshold = 128;
const int angle_fix_max = 1024;

// Residual for the angle integration; needed because the speed is higher resolution than the angle.
int angle_residual = 0;

// Keep the active output as ThreePhase to avoid a conversion.
ThreePhase motor_outputs = {0, 0, 0};

// Our outputs are delayed 1 cycle; store the previous outputs here before we use them.
ThreePhase previous_motor_outputs = {0, 0, 0};

int previous_emf_angle_error = 0;

int32_t rotor_acceleration_observer = 0;

int32_t angular_speed_observer = 0;

int16_t angle_adjustment_residual = 0;

// Start with the missing hall sector marker.
uint8_t previous_hall_sector = hall_sector_base;

int32_t resistive_power_observer = 0;

int32_t total_power_observer = 0;


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

PositionCalibration position_calibration = get_position_calibration();
CurrentCalibration current_calibration = get_current_calibration();
ControlParameters control_parameters = get_control_parameters();

// Guard the data access by disabling the ADC interrupt while we read/write the data.

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

void set_angle(int16_t angle) {
    // Disable the ADC interrupt while we modify the readout angle.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    readout.angle = normalize_angle(angle);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}


// Helper funcs
// ------------


// Combine the motor duty cycles into a single 32-bit value; because it barely fits.
static inline uint32_t encode_pwm_commands(ThreePhase const& outputs){
    return (
        std::get<0>(outputs) * pwm_base * pwm_base +
        std::get<1>(outputs) * pwm_base +
        std::get<2>(outputs)
    );
}



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
    if (not LL_ADC_IsActiveFlag_JEOS(ADC1)) return set_motor_outputs(breaking_motor_outputs);

    
    // U and W phases are measured at the same time, followed by V and the reference voltage.
    // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
    // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
    // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
    
    // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
    // The PWM period lasts 1/23.4KHz = 42.7us.

    // Read the reference voltage first; this is the voltage of the reference line for the current sense amplifier.
    const uint16_t ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3);

    // I wired the shunt resistors in the wrong way, so we need to flip the sign of the current readings.
    // Flip the sign of V because we accidentally wired it the other way (the correct way...). Oopsie doopsie.
    const ThreePhase readouts = {
        -(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) - ref_readout),
        +(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) - ref_readout),
        -(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) - ref_readout)
    };

    // Note that the reference voltage is only connected to the current sense amplifier, not the
    // microcontroller. The ADC reference voltage is 3.3V.

    // Also read the controller chip temperature.
    const uint16_t instant_temperature = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);

    // And motor supply voltage.
    const uint16_t instant_vcc_voltage = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);


    // Get hall sensor state
    // ---------------------

    // Read new data from the hall sensors.
    const uint8_t hall_state = read_hall_sensors_state();

    // There are 3 hall sensors each getting turned on by a positive magnetic field. When 2 sensors
    // are on at the same time, we infer that the rotor is halfway between the two sensors. When a
    // single sensor is on the rotor north must be very close to that sensor. This divides the circle
    // into 6 sectors, we number the sectors from 0 to 5 with 0 indicating rotor north over hall sensor U.
    const uint8_t hall_sector = get_hall_sector(hall_state);

    // Use `hall_sector_base` to flag for invalid hall sensor readings. This happens when no sensors are
    // active, or if they are all on for some reason (probably the sensor type).
    const bool hall_valid = hall_sector < hall_sector_base;

    // Calculate the angle of the hall reading in our angle units.
    const int hall_angle = hall_sector * hall_sector_span;


    // Do the data calculations
    // ------------------------

    // Average the temperature readings since we are sampling quicker than the manufacturer indicates.
    // We can't extend the sampling time longer than it is set at the moment (about half the recommendation),
    // so we have to massage the readings for noise. Temperature varies slowly anyway.
    const int temperature = (instant_temperature + readout.temperature * 3) / 4;

    // Average out the VCC voltage; it should be relatively stable so we average to reduce our error.
    const int vcc_voltage = (instant_vcc_voltage + readout.vcc_voltage * 3) / 4;

    // Calculate our outputs on the motor phases. The outputs are delayed by one cycle and then
    // quickly averaged over time (half life of 1 cycle). The averaging is masking some effects
    // from the inductance, but averaging like this greatly reduces the noise in inferred EMF
    // readings so... we keep it.
    motor_outputs = (previous_motor_outputs + motor_outputs) / 2;

    // Store the active motor outputs for the next cycle. 
    previous_motor_outputs = get_duties(driver_state.motor_outputs);

    // Calculate calibrated currents.
    const ThreePhase currents = readouts * get_calibration_factors(current_calibration) / current_calibration_fixed_point;

    // Get calibrated current divergence (the time unit is defined as 1 per cycle). We average out the
    // current diffs exactly as we do with the motor outputs; this seems to work best, can't explain why.
    //
    // I've attempted to use the finite differences approach `diff(x) = ((x[n] - x[n-1]) + (x[n+1] - x[n])) / (2 * dt)`
    // but it doesn't work as well as averaging both motor outputs and single step current diff.
    const ThreePhase currents_diff = ((currents - get_currents(readout)) + get_currents_diff(readout)) / 2;

    // Calculate the voltage drop across the coil inductance.
    const ThreePhase inductor_voltages = currents_diff * current_calibration.inductance_factor / phase_diff_conversion;

    // Calculate the resistive voltage drop across the coil and MOSFET resistance.
    const ThreePhase resistive_voltages = currents * phase_current_to_voltage / current_fixed_point;

    // Calculate the driven phase voltages from our PWM settings and the VCC voltage. We adjust our voltages
    // such that the 0 point corresponds to the voltage at the connection point of the three phases. The
    // motor stator coils are usually connected together by the manufacturer for a star configuration motor.
    const ThreePhase drive_voltages = adjust_to_sum_zero(motor_outputs) * vcc_voltage / pwm_base;
    
    // Infer the back EMF voltages for each phase.
    // 
    // Calculate the EMF voltage as the remainder after subtracting the electric circuit voltages.
    // By Kirchoffs laws the total voltage of all of our components must sum to 0.
    const ThreePhase emf_voltages = inductor_voltages + resistive_voltages - drive_voltages;


    // Position Update
    // ---------------

    // Flip the rotor if we have reached the threshold number of incorrect detections.
    const bool rotor_direction_flip = incorrect_direction_detections >= control_parameters.incorrect_direction_threshold;

    // Flip the rotor angle if we have reached the threshold number of incorrect detections.
    // 
    // Note: do this before computing any DQ0 values as they should be consistent with the angle.
    if (rotor_direction_flip) {
        // It should be normalized below.
        readout.angle += half_circle;

        // Reset the incorrect direction counter on flip.
        incorrect_direction_detections = 0;
    }


    // Predict the position; keeping track of fractional angles at the same resolution as
    // the speed. By our definition the time unit is 1 per cycle; so the angle spanned by 
    // the rotor is exactly the angular speed.
    const int angle_hires_diff = readout.angular_speed + angle_residual;

    // Advance the angle and the angle residual.
    const int unnormalized_predicted_angle = readout.angle + angle_hires_diff / speed_fixed_point;
    
    // Keep track of the remaining angle fraction spanned according to the angular speed for which the integer angle
    // doesn't change. Hopefully the compiler makes the % operation free considering it follows the division above.
    angle_residual = angle_hires_diff % speed_fixed_point;

    // Compute and normalize the new rotor angle.
    const int predicted_angle = normalize_angle(unnormalized_predicted_angle);


    // Switching to DQ0 Frame
    // ----------------------

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

    const int direct_current = dot(currents, three_phase_cos) / angle_base;

    const int quadrature_current = -dot(currents, three_phase_sin) / angle_base;

    const int direct_emf_voltage = dot(emf_voltages, three_phase_cos) / angle_base;

    const int quadrature_emf_voltage = -dot(emf_voltages, three_phase_sin) / angle_base;


    // Current angle calculation
    // -------------------------

    // Calculate the angle at which the current is running on the motor coils. The angle offset is
    // with respect to the predicted angle as that was the angle used in the park transform.
    const int inductor_angle_offset = funky_atan2(quadrature_current, direct_current);
    
    // Current angle in the stator frame of reference.
    const int inductor_angle = normalize_angle(predicted_angle + inductor_angle_offset);
    
    // Calculate the magnitude by rotating the current vector entirely on the quadrature axis.
    const int instant_current_magnitude = faster_abs(
        get_cos(inductor_angle_offset) * direct_current + 
        get_sin(inductor_angle_offset) * quadrature_current
    ) / angle_base;

    // The current measurements have a low noise floor, but it's not 0.
    const bool current_detected = instant_current_magnitude > 4;

    // Average the current magnitude over a short duration to reduce noise.
    const int current_magnitude = (instant_current_magnitude + readout.current_magnitude * 3) / 4;
    

    // Back EMF angle observer
    // -----------------------

    // Get the angle measured from EMF relative to the predicted rotor angle.
    const int emf_angle_offset = funky_atan2(direct_emf_voltage, -quadrature_emf_voltage);

    // Calculate the emf voltage as a rotation of the quad voltage that zeroes out the direct component.
    const int instant_emf_voltage_magnitude = faster_abs(
        get_cos(emf_angle_offset) * quadrature_emf_voltage - 
        get_sin(emf_angle_offset) * direct_emf_voltage
    ) / angle_base;

    // Average the EMF voltage magnitude over a short duration to reduce noise.
    const int emf_voltage_magnitude = (instant_emf_voltage_magnitude + readout.emf_voltage_magnitude * 3) / 4;

    // The rotor angle inferred from the EMF can be either aligned with the positive quadrature direction or 
    // the negative. It's going to be aligned with the negative direction when we have the rotor switches
    // from positive to negative speed. The angle becomes extremely noisy at standstill (crossing 0 speed).
    const int emf_angle_error = angle_or_mirror(emf_angle_offset);

    // Measure the noise of the angle error. We can't rely on the measured error above the configured noise threshold.
    const int emf_angle_error_variance = min(
        max_16bit,
        1 + (square(emf_angle_error - previous_emf_angle_error) + readout.emf_angle_error_variance * 3) / 4
    );

    // Store the current error for the noise calculation next cycle.
    previous_emf_angle_error = emf_angle_error;

    // Check if the EMF voltage is away from zero with enough confidence.
    const bool emf_detected = (
        (emf_angle_error_variance < control_parameters.emf_angle_error_variance_threshold) and
        (instant_emf_voltage_magnitude > control_parameters.min_emf_voltage)
    );

    // Keep track of how many EMF detections we have in a row.
    number_of_emf_detections = clip_to(0, emf_fix_max, number_of_emf_detections + (emf_detected ? +1 : -1));

    // Let the angle adjust a few steps before using the diff to compute the speed; our initial guess starts
    // at an arbitrary position so the apparent acceleration is just the angle converging to the correct value.
    const bool compute_speed = number_of_emf_detections >= emf_speed_threshold;

    // Declare that we have an emf reading after enough detections.
    const bool emf_fix = number_of_emf_detections >= emf_fix_threshold;

    // Check if we have the incorrect rotor angle by checking if the quad EMF voltage has opposite sign to the angular speed.
    const bool incorrect_rotor_angle_detected = emf_fix and (quadrature_emf_voltage * readout.angular_speed > 32);

    // Keep track of the number of times we detected an incorrect direction for the speed and EMF sign.
    incorrect_direction_detections = max(0, incorrect_direction_detections + (incorrect_rotor_angle_detected ? +1 : -1));

    // Set the flag for immininent rotor correction, we need to set it over multiple cycles otherwise it might be missed.
    const bool rotor_direction_flip_imminent = incorrect_direction_detections >= incorrect_direction_warning;

    // Our angle is incorrect if we're detecting a flip; or if we don't have an EMF reading whilst driving the motor.
    const bool incorrect_angle = incorrect_direction_detections or (driver_state.active_pwm and not emf_fix);

    // Track how many times we think our rotor angle is correct. Note that we keep the angle fix whilst the motor is off.
    correct_angle_counter = clip_to(
        0, angle_fix_max, 
        // Subtract 1 for incorrect angles; otherwise add 1 for emf or hall angle fixes.
        correct_angle_counter + (incorrect_angle ? -1 : emf_fix) + (hall_valid * control_parameters.hall_angle_ki > 0 ? +1 : 0)
    );

    // If the angle error is between -90 and +90 degrees, use it directly otherwise use the mirror angle.
    const int prediction_error = emf_detected * emf_angle_error;
    

    // Angle update
    // ------------

    // When we don't have an angle estimate from the back EMF, then calculate the angle adjustment from
    // the hall sensors when the hall sensors are valid.
    const int hall_prediction_error = (not emf_detected) * hall_valid * signed_angle(hall_angle - predicted_angle);

    // Declare the angle to be correct after a threshold certainty.
    const bool angle_fix = correct_angle_counter >= angle_fix_threshold;
    
    // Calculate the angle adjustment error using the parametrized gains. The
    // hall gain can be set to 0 to disable the hall sensor angle correction.
    const int angle_adjustment_hires = (
        angle_adjustment_residual +
        prediction_error * control_parameters.rotor_angle_ki +
        hall_prediction_error * control_parameters.hall_angle_ki
    );
    
    // Use the same high resolution trick as for the `angle_residual` to accumulate fractional adjustments.
    const int angle_adjustment = angle_adjustment_hires / hires_fixed_point;

    // The compiler should optimize the modulo after the same division above.
    angle_adjustment_residual = angle_adjustment_hires % hires_fixed_point;
    
    // Calculate the new angle based on the angle adjustment.
    const int unnormalized_angle = unnormalized_predicted_angle + angle_adjustment;

    // Increment rotations if we have moved outside the 0 to 2*pi range.
    const int rotations_increment = (
        unnormalized_angle < 0 ? -1 : 
        unnormalized_angle > angle_base ? +1 : 
        0
    );

    // Calculate the new angle and keep it normalized using the rotations calculation.
    const int angle = unnormalized_angle - rotations_increment * angle_base;

    // Calculate the new rotation index.
    const int unnormalized_rotations = readout.rotations + rotations_increment;

    // Normalize the rotations to the 16-bit range. Note that we don't use the minimum
    // negative 16 bit value so that calculations with the negative of rotations are still
    // in the valid range for signed 16 bit arithmetic.
    const int rotations = (
        unnormalized_rotations > max_16bit ? -max_16bit : 
        unnormalized_rotations < -max_16bit ? +max_16bit : 
        unnormalized_rotations
    );


    // Calculate speed and acceleration
    // --------------------------------

    // Calculate the new speed based on the angle adjustment.
    // 
    // Note that the angle change is relative to the current speed because of the prediction step.
    const int speed_adjustment = (
        // If we have enough EMF detections, adjust the speed according to the prediction error.
        compute_speed ? speed_fixed_point * prediction_error * control_parameters.rotor_angular_speed_ki :
        // Maintain speed if we have an EMF reading, even if noisy.
        emf_detected ? 0 :
        // Otherwise drop the speed to 0.
        -angular_speed_observer
    );
    
    // Clamp the speed observer such that the low resolution speed is clamped to `max_angular_speed`.
    angular_speed_observer = clip_to(
        -max_angular_speed_observer, 
        +max_angular_speed_observer, 
        angular_speed_observer + speed_adjustment
    );

    const int angular_speed = angular_speed_observer / hires_fixed_point;
    
    // Calculate the acceleration based on the speed change. We can use gradient descent to slowly
    // decrease our speed error. Equivalent to an exponential moving average, however framing it as
    // a gradient descent allows us to integrate the error into a higher resolution observer.
    const int acceleration_error = ((angular_speed - readout.angular_speed) * acceleration_fixed_point - readout.rotor_acceleration);

    // Update the high resolution observer for the rotor acceleration.
    rotor_acceleration_observer += acceleration_error * control_parameters.rotor_acceleration_ki;

    const int rotor_acceleration = rotor_acceleration_observer / hires_fixed_point;


    // Calculate the power use
    // -----------------------

    // Resistive power is the power dissipated in the motor coils and MOSFETs.
    const int resistive_power = dot(currents, resistive_voltages) / voltage_current_div_power_fixed_point;

    // Inductive power is the power transfered to the motor inductors.
    const int inductive_power = dot(currents, inductor_voltages) / voltage_current_div_power_fixed_point;

    // EMF power is the power transferred into the rotor movement, driving the motor.
    // 
    // Use the DQ0 transformed values to calculate the EMF power quickly. We also have a chance to 
    // smooth out the values to better approximate the real power use.
    const int emf_power = - (
        sign(quadrature_current) * current_magnitude * 
        sign(quadrature_emf_voltage) * emf_voltage_magnitude 
    ) / dq0_to_power_fixed_point;

    // The total power is the power used from the battery. It will be positive when driving
    // the motor, meaning that we drain the battery. If this is negative it means we are charging
    // the battery by slowing down the motor (regenerative breaking).
    // 
    // The balance of all powers must be zero assuming no other source or sink of power. Thus
    // we can compute the total power from the others; mostly determined by EMF. The resistive
    // power is quite reliable and inductive_power is very small.
    const int total_power = resistive_power + inductive_power + emf_power;


    // Cap the maximum PWM
    // -------------------

    // Calculate slowly varying averages of the resistive power; this represents the energy
    // dissipated in the motor coils which should be proportional to the temperature rise.
    const int avg_resistive_power = resistive_power_observer / hires_fixed_point;
    
    // Update the higher resolution observer.
    resistive_power_observer += (resistive_power - avg_resistive_power) * control_parameters.resistive_power_ki;

    // Calculate slowly varying averages of the total power; this represents the energy
    // drawn from the battery. At constant voltage, this is proportional to the current drawn.
    const int avg_total_power = total_power_observer / hires_fixed_point;

    // Update the higher resolution observer.
    total_power_observer += (total_power - avg_total_power) * control_parameters.power_draw_ki;


    // Reduce the maximum output PWM to keep within safe limits:
    // 1. The MOSFET drivers need to be kept in their operating voltage range. Reduce PWM to
    // let the battery recharge our local capacitors.
    // 2. The resistive power heats up the motor coils. Keep it under a threshold to avoid overheating.
    // 3. The total power is a good proxy for total current consumed from the battery.
    // 
    // +limiting_divisor_m1 so we do ceiling of the division.
    // 
    // The penalty should normally be negative indicating we can increase the PWM.
    const int pwm_penalty = (max(
        vcc_mosfet_driver_undervoltage - vcc_voltage,
        avg_resistive_power - control_parameters.max_resistive_power,
        avg_total_power - control_parameters.max_power_draw,
        faster_abs(angular_speed) - control_parameters.max_angular_speed
    ) + limiting_divisor_m1) / limiting_divisor;

    const int live_max_pwm = clip_to(0, control_parameters.max_pwm, readout.live_max_pwm - pwm_penalty);



    // Write the latest readout data
    // -----------------------------
    // 
    // Must be updated before the motor pwm calculation!

    readout.pwm_commands = encode_pwm_commands(motor_outputs);
    
    readout.readout_number = readout_number;
        
    readout.state_flags = (
        (emf_fix << emf_fix_bit_offset) |
        (emf_detected << emf_detected_bit_offset) |
        (current_detected << current_detected_bit_offset) |
        (angle_fix << angle_fix_bit_offset) |
        (incorrect_rotor_angle_detected << incorrect_rotor_angle_bit_offset) |
        (rotor_direction_flip_imminent << rotor_direction_flip_imminent_bit_offset) |
        (hall_state << hall_state_bit_offset)
    );

    
    readout.u_current = std::get<0>(currents);
    readout.v_current = std::get<1>(currents);
    readout.w_current = std::get<2>(currents);

    readout.ref_readout = ref_readout;
    
    readout.u_current_diff = std::get<0>(currents_diff);
    readout.v_current_diff = std::get<1>(currents_diff);
    readout.w_current_diff = std::get<2>(currents_diff);

    readout.angle = angle;

    readout.angle_adjustment = angle_adjustment;
    readout.angular_speed = angular_speed;
    readout.vcc_voltage = vcc_voltage;
    readout.emf_voltage_magnitude = emf_voltage_magnitude;

    readout.temperature = temperature;
    readout.live_max_pwm = live_max_pwm;

    readout.direct_current = direct_current;
    readout.quadrature_current = quadrature_current;
    readout.direct_emf_voltage = direct_emf_voltage;
    readout.quadrature_emf_voltage = quadrature_emf_voltage;
    
    readout.total_power = total_power;
    readout.resistive_power = resistive_power;
    readout.emf_power = emf_power;
    readout.inductive_power = inductive_power;
    
    readout.inductor_angle = inductor_angle;

    readout.rotor_acceleration = rotor_acceleration;
    readout.rotations = rotations;

    readout.emf_angle_error_variance = emf_angle_error_variance;
    readout.current_magnitude = current_magnitude;
    
    readout.lead_angle = driver_state.lead_angle;
    readout.target_pwm = driver_state.target_pwm;
    
    readout.secondary_target = driver_state.secondary_target;
    readout.seek_integral = driver_state.seek_angle.error_integral / seek_integral_divisor;
    

    // Calculate motor outputs
    // -----------------------

    // Setup the new state if we were commanded by the main loop.
    if (new_pending_state) {
        driver_state = setup_driver_state(driver_state, pending_state, readout);
        new_pending_state = false;
    }

    // Update the motor controls using the readout data.
    update_motor_control(driver_state, readout);

    // Try to write the latest readout snippet if there's space.
    readout_history_push(readout);


    // Set Motor Outputs!!
    // -------------------

    // Disable the update for the control registers so we can write all 3.
    LL_TIM_DisableUpdateEvent(TIM1);

    // Send the command to the timer compare registers. Set the registers close to when cycle_end_tick 
    // is set so we can properly track the value for the next cycle. There's a half cycle delay if
    // we set the output registers too late in the cycle.
    set_motor_outputs(driver_state.motor_outputs);

    // Re-enable the update for the control registers now that we've written all 3.
    LL_TIM_EnableUpdateEvent(TIM1);

    readout.cycle_end_tick = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP ? LL_TIM_GetCounter(TIM1) : (pwm_period - LL_TIM_GetCounter(TIM1));

    // Clear the ADC end of conversion flag so we're ready for the next conversion.
    LL_ADC_ClearFlag_JEOS(ADC1);
}


// Other interrupt handlers
// ------------------------

// These functions are called by the autogenerated C code.


void tim1_update_interrupt_handler(){
    // We shouldn't trigger this, but including for documentation.
    return set_motor_outputs(breaking_motor_outputs);
    
    // Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
    
    // Note, this updates on both up and down counting, get direction 
    // with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
}


void tim2_global_handler(){
    // We shouldn't trigger this, but including for documentation.
    return set_motor_outputs(breaking_motor_outputs);

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


// Initialization
// --------------

// We might need this to make sure we can quickly read the sin and phase tables (make
// sure they are loaded from flash to RAM). Not sure if this does anything though...

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
