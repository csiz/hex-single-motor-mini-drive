#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "main.h"

#include "constants.hpp"
#include "motor_control.hpp"
#include "interrupts.hpp"

#include <stm32f1xx_ll_gpio.h>



extern uint32_t adc_update_number;
extern uint32_t tim1_update_number;
extern uint32_t tim2_update_number;
extern uint32_t tim2_cc1_number;

// Electrical state
// ----------------

struct UpdateReadout{
    uint16_t readout_number;
    uint16_t position;
    uint32_t pwm_commands;
    uint16_t u_readout;
    uint16_t v_readout;
    uint16_t w_readout;
    uint16_t ref_readout;
};

extern UpdateReadout latest_readout;
const size_t READOUT_ITEMSIZE = sizeof(UpdateReadout);

extern QueueHandle_t readouts_queue;
extern StaticQueue_t readouts_queue_storage;
extern uint8_t readouts_queue_buffer[HISTORY_SIZE * READOUT_ITEMSIZE];
extern uint32_t readouts_missed;

extern bool readouts_allow_missing;
extern bool readouts_allow_sending;

extern int32_t readouts_to_send;

// Hall sensors
// ------------

// Hall states as bits, 0b001 = hall 1, 0b010 = hall 2, 0b100 = hall 3.
extern uint8_t hall_state;
// The 6 valid configurations of hall sensors in trigonometric order.
extern uint8_t hall_sector;
// Whether the sector is valid; it will be false if the magnet is not present or the sensors are not ready.
extern bool hall_sector_valid;

// Position tracking
// -----------------

// Some useful functions to compute position. These need to be inlined for 
// efficiency, and constexpr to define a bunch more constants below.

// Square a number.
inline constexpr int square(int x){
    return x * x;
}

// Get the smaller between two numbers.
inline constexpr int min(int a, int b){
    return a < b ? a : b;
}

// Get the larger between two numbers.
inline constexpr int max(int a, int b){
    return a > b ? a : b;
}

// Clip a value between two limits; params are (low high value).
inline constexpr int clip_to(int low, int high, int value){
    return min(high, max(low, value));
}

// ### Constants
//
// Define a lot of constants. First, we need to define angles as integers because 
// we can't (shouldn't) use floating point arithmetic in the interrupt handlers.
// Then define the time units. Then speed units, further scaled by a constant to
// have enough precision to represent it.

// Maximum value we can use in signed 32 bit multiplication.
const int max_16bit = 0x0000'8FFF; //0x0000'B504;

// Angle units of a full circle (2pi).
const int max_angle_unit = 2048;
// Half a circle (pi).
const int half_max_angle = max_angle_unit / 2;
// 3/2 of a circle (3pi/2).
const int one_and_half_max_angle = (3 * max_angle_unit) / 2;

// Normalize to a positive angle (0 to 2pi).
inline constexpr int normalize_angle(int angle){
    return (angle + max_angle_unit) % max_angle_unit;
}
// Normalize a 0 centerd angle; keeping its sign (-pi to pi).
inline constexpr int signed_angle(int angle){
    return (angle + one_and_half_max_angle) % max_angle_unit - half_max_angle;
}

// Scaling constant for time.
const int ticks_per_time_units = 256;
// Ticks per microsecond with the 72MHz clock.
const int ticks_per_microsecond = 72;
// Ticks per millisecond with the 72MHz clock.
const int ticks_per_millisecond = 72'000;
// Our time units per millisecond. Helpful scaling factor for the constants below.
const int time_units_per_millisecond = ticks_per_millisecond / ticks_per_time_units; 
// Time units per PWM cycle (2x because it counts up then down).
const int time_increment_per_cycle = 2 * static_cast<int>(PWM_BASE) / ticks_per_time_units;

// Another scaling factor: speed = distances * scale / time; acceleration = speed_change * scale;
const int scale = 128;
// Precomputed square of scale.
const int square_scale = square(scale);

// Reference for the maximum speed we should be able to represent.
const int max_speed = 20 * scale * max_angle_unit / time_units_per_millisecond;

// The maximum time in our time units before we can no longer safely square the value.
// 
// Keep in mind that a single motor rotation takes at least 12 toggles (6 per electrical
//  revolution * 2 poles per phase).
const int max_time_between_observations = 100 * time_units_per_millisecond;




// Speed and acceleration are written in degrees per ms and per ms^2 respectively.

// Initial speed estimate.
const int initial_angular_speed = 0;
// Start with a high speed variance.
const int initial_angular_speed_variance = square(scale * max_angle_unit * 30 / 360 / time_units_per_millisecond);

// Precalculate the acceleration variance divided by 4. Note the scaled is squared twice.
const int angular_acceleration_variance_div_4 = square(square_scale * max_angle_unit / 1 / 50 / time_units_per_millisecond / time_units_per_millisecond) / 4;

// Maximum distance to a trigger angle. Don't let the estimated angle deviatie
// from the hall sesor angle by more than this value to keep the estimate
// within the half circle of the trigger so we don't switch sign.
const int sector_transition_confidence = 20 * max_angle_unit / 360;

// Variance of the hall sensor; it doesn't seem to be consistent, even between two rotations.
const int default_sector_transition_variance = square(5 * max_angle_unit / 360);
// Variance of a gaussian spread over the entire sector.
const int default_sector_center_variance = square(30 * max_angle_unit / 360);

// The hall sensors trigger later than expected going each direction.
const int hysterisis = 5 * max_angle_unit / 360;

// The angle at which we transition to this sector. The first is when rotating in the
// positive direction; second for the negative direction.
const int sector_transition_angles[6][2] = {
    {330 * max_angle_unit / 360 + hysterisis,  30 * max_angle_unit / 360 - hysterisis},
    { 30 * max_angle_unit / 360 + hysterisis,  90 * max_angle_unit / 360 - hysterisis},
    { 90 * max_angle_unit / 360 + hysterisis, 150 * max_angle_unit / 360 - hysterisis},
    {150 * max_angle_unit / 360 + hysterisis, 210 * max_angle_unit / 360 - hysterisis},
    {210 * max_angle_unit / 360 + hysterisis, 270 * max_angle_unit / 360 - hysterisis},
    {270 * max_angle_unit / 360 + hysterisis, 330 * max_angle_unit / 360 - hysterisis},     
};

// Variance of each sector transition; we can calibrate it.
const int sector_transition_variances[6][2] = {
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
    {default_sector_transition_variance, default_sector_transition_variance},
};

// The center of each hall sector; the motor should rest at these poles.
const int sector_center_angles[6] = {
    (  0 * max_angle_unit / 360),
    ( 60 * max_angle_unit / 360),
    (120 * max_angle_unit / 360),
    (180 * max_angle_unit / 360),
    (240 * max_angle_unit / 360),
    (300 * max_angle_unit / 360),
};

// Variance of the centers.
const int sector_center_variances[6] = {
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
    default_sector_center_variance,
};

// ### Position tracking variables

// Whether the position is valid.
extern bool angle_valid;
// We need to keep track of the previous hall sector to determine the direction of rotation.
extern uint8_t previous_hall_sector;
// Whether the previous hall sector was valid; initialize anew when the magnet is placed back.
extern bool previous_hall_sector_valid;

// Time, in our units, since the last observation of the hall sensor. The PWM cycle loop updates
// this variable.
extern int time_since_observation;
// Flag to indicate a new observation; the hall sensor interrupt sets the flag for the PWM cycle loop.
extern bool new_observation;

// Estimated angle at the last observation; this is our best guess of the angle using a kalman filter.
extern int angle_at_observation;
// Variance of the angle at the last observation.
extern int angle_variance_at_observation;
// Estimated angular speed at the last observation; this is our best guess of the speed using a kalman filter.
extern int angular_speed_at_observation;
// Variance of the angular speed at the last observation.
extern int angular_speed_variance_at_observation;



// Motor Currents
// --------------

// Computed motor phase currents.
extern float current_u, current_v, current_w;

// Voltage reference for the ADC; it's a filtered 3.3V that power the board.
const float adc_voltage_reference = 3.3;
// Shunt resistance for the motor phase current sensing are 10mOhm, 500mW resistors.
const float motor_shunt_resistance = 0.010;
// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
const float amplifier_gain = 20.0;
// The ADC has a 12-bit resolution.
const uint16_t adc_max_value = 0xFFF; // 2^12 - 1 == 4095 == 0xFFF.
// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
// And the conversion from the ADC readout is given by:
//   Vout = adc_voltage_reference * (adc_current_readout / adc_max_value).
// So the current is:
//   Iload = (Vout - Vref) / (Rsense * GAIN) = adc_voltage_reference * (adc_readout_diff / adc_max_value) / (Rsense * GAIN).
// 
// Note: The minus sign is because of the way I wired up the INA4181 ...
const float current_conversion = -adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);



// Functions
// ---------

// Initialize the data structures.
void data_init();

// Compute motor phase currents using latest ADC readouts.
void calculate_motor_phase_currents_gated();



// Read the hall sensors and update the motor rotation angle. Sensor chips might be: SS360NT (can't read the inprint clearly).
static inline void read_hall_sensors(){
    // Grab the registers for the GPIO ports with the hall sensors.

    uint16_t gpio_A_inputs = LL_GPIO_ReadInputPort(GPIOA);
    uint16_t gpio_B_inputs = LL_GPIO_ReadInputPort(GPIOB);

    // Note: Hall sensors are active low!
    const bool hall_1 = !(gpio_A_inputs & (1<<0)); // Hall sensor 1, corresponding to phase V
    const bool hall_2 = !(gpio_A_inputs & (1<<1)); // Hall sensor 2, corresponding to phase W
    const bool hall_3 = !(gpio_B_inputs & (1<<10)); // Hall sensor 3, corresponding to phase U

    // Combine the hall sensor states into a single byte.
    // Note: Reorder the sensors according to the phase order; U on bit 0, V on bit 1, W on bit 2.
    hall_state = hall_3 | (hall_1 << 1) | (hall_2 << 2);

    // Get the hall sector from the state.
    switch (hall_state) {
        case 0b000: // no hall sensors; either it's not ready or no magnet
            hall_sector_valid = false;
            break;
        case 0b001: // hall U active; 0 degrees
            hall_sector = 0;
            hall_sector_valid = true;
            break;
        case 0b011: // hall U and hall V active; 60 degrees
            hall_sector = 1;
            hall_sector_valid = true;
            break;
        case 0b010: // hall V active; 120 degrees
            hall_sector = 2;
            hall_sector_valid = true;
            break;
        case 0b110: // hall V and hall W active; 180 degrees
            hall_sector = 3;
            hall_sector_valid = true;
            break;
        case 0b100: // hall W active; 240 degrees
            hall_sector = 4;
            hall_sector_valid = true;
            break;
        case 0b101: // hall U and hall W active; 300 degrees
            hall_sector = 5;
            hall_sector_valid = true;
            break;
        case 0b111: // all hall sensors active; this would be quite unusual
            hall_sector_valid = false;
            Error_Handler();
            break;
    }
}

// Update position estimate after not observing any trigger for a while.
static inline void update_position_unobserved(){
    if (not angle_valid) return;
    const int time = max(1, time_since_observation);

    const int direction = angular_speed_at_observation >= 0 ? 1 : -1;
    const size_t direction_index = angular_speed_at_observation >= 0 ? 0 : 1;

    const int next_sector = (6 + hall_sector + direction) % 6;

    const int trigger_angle = sector_transition_angles[next_sector][direction_index];

    const int distance_to_trigger = signed_angle(trigger_angle - angle_at_observation);
    
    const int max_abs_speed = direction * (distance_to_trigger + direction * sector_transition_confidence) * scale / time;

    angular_speed_at_observation = direction * clip_to(0, max_abs_speed, direction * angular_speed_at_observation);

    if (time_since_observation >= max_time_between_observations) {
        // We haven't observed the hall sensors for a while; we need to reset the position estimate.
        angle_at_observation = sector_center_angles[hall_sector];
        angle_variance_at_observation = sector_center_variances[hall_sector];
        angular_speed_at_observation = initial_angular_speed;
        angular_speed_variance_at_observation = initial_angular_speed_variance;
    }
}

// Update the position from the hall sensors. Use a kalman filter to estimate the position and speed.
static inline void update_position_observation(){
    // Get the time since the last observation.
    const int time = max(1, time_since_observation);

    read_hall_sensors();

    // Check if the magnet is present.
    if (not hall_sector_valid) {
        previous_hall_sector_valid = false;
        angle_valid = false;
        return;
    }

    // The new hall sector is valid, let's calculate the angle.

    if (not previous_hall_sector_valid || hall_sector == previous_hall_sector) {
        // This is the first time we have a valid hall sector; we need to set the angle and the angular speed.
        previous_hall_sector = hall_sector;
        previous_hall_sector_valid = true;

        // Reset position tracking to the sector center; our best guess.

        angle_at_observation = sector_center_angles[hall_sector];
        angle_variance_at_observation = sector_center_variances[hall_sector];
        angular_speed_at_observation = initial_angular_speed;
        angular_speed_variance_at_observation = initial_angular_speed_variance;

        new_observation = true;
        angle_valid = true;
        return;
    }

    
    // We have a valid sector transition; perform a Kalman filter update.

    // Get the sector integer distance to determine the direction of rotation.
    const int sector_distance = (9 + hall_sector - previous_hall_sector) % 6 - 3;

    // Establish direction of rotation.
    const int direction = sector_distance >= 0 ? 1 : -1;
    // Positive rotations index the first element in the calibration table, negative the second.
    const size_t direction_index = sector_distance >= 0 ? 0 : 1;

    // Get data about this transition from the calibration table.

    const int trigger_angle = sector_transition_angles[hall_sector][direction_index];
    const int trigger_variance = sector_transition_variances[hall_sector][direction_index];
    const int sector_variance = sector_center_variances[hall_sector];

    // Change coordinates with angle as the center. The next trigger is always close to the current 
    // angle (max distance between sectors is 120 degrees). On the other hand the distance since
    // the last measurement can grow arbitrarily large, we need to be careful to not wrap the estiamte
    // around the circle, but we do need to wrap and normalize the distance to the trigger.
    // 
    // Note: Don't add up variances for the change of coordinates! It's just a math trick.

    // Calculate the distance travelled since the last observation to the current trigger angle.
    const int distance_to_trigger = signed_angle(trigger_angle - angle_at_observation);
    
    // Make sure we don't overshoot the upper bound of the trigger angle. This is likely because
    // our speed doesn't update between observations and a long time can pass until we get a new one.
    // A low residual speed will push us quite a bit over a long enough time. Cap the speed based
    // on the observation time such that we don't overshoot the trigger angle by more than the sector confidence.

    // Calculate the maximum allowed speed while handling the direction sign.
    const int max_abs_speed = direction * (distance_to_trigger + direction * sector_transition_confidence) * scale / time;

    // Clip the angular speed to the maximum allowed speed; we also clip it so it does't go backwards. A positive
    // sector transition necesarily implies a positive speed of the rotor.
    const int clipped_angular_speed = direction * clip_to(0, max_abs_speed, direction * angular_speed_at_observation);

    // The distance traveled is the speed * time divided by our extra scaling factor (for integer math precision).
    const int estimated_distance = clipped_angular_speed * time / scale;
    
    // Get the error in our estimate.
    const int estimated_distance_error = distance_to_trigger - estimated_distance;

    // Get the speed error.
    const int estimated_speed_error = scale * estimated_distance_error / time;
    
    // Precompute the square of time and downscale by our magic scale to keep it within integer arithmetic bounds.
    const int square_time_div_square_scale = clip_to(1, max_16bit, time * time / square_scale);

    // Calculate the variances.

    // The angular speed variance increases with acceleration over time.
    const int angular_speed_variance = min(max_16bit,
        angular_speed_variance_at_observation + angular_acceleration_variance_div_4 * square_time_div_square_scale);
    
    // The distance variance does not depend on the trigger variance as we're treating it as a coordinate change.
    // This is the variance of the estimated angle! It depends on the last angle and speed variances.
    const int estimated_distance_variance = min(max_16bit, 
        angle_variance_at_observation + square_time_div_square_scale * angular_speed_variance);
    
    // Variance of the speed error. This is the variance of our predicted speed. It depends on the estimate of the
    // last angle, the trigger angle, and the previous speed variance.
    const int estimated_speed_error_variance = min(max_16bit, 
        angular_speed_variance + (angle_variance_at_observation + trigger_variance) / square_time_div_square_scale);

    // Adjust the distance using the kalman gain. This is actually the new mean of a product of gaussians.
    const int distance_adjustment = (estimated_distance_error * estimated_distance_variance) / (estimated_distance_variance + trigger_variance);
    // Similarly adjust the speed based on our new guess and previous variance.
    const int speed_adjustment = (estimated_speed_error * angular_speed_variance) / (angular_speed_variance + estimated_speed_error_variance);
    
    // Return to the original coordinates and use the distance adjustment computed with the kalman gain.
    const int kalman_angle = normalize_angle(angle_at_observation + estimated_distance + distance_adjustment);
    
    // Update the position parameters.
    
    // Store the previous hall sector and mark it as valid.
    previous_hall_sector = hall_sector;
    previous_hall_sector_valid = true;
    
    // Ensure the angle at observation is within the confidence band of the current sector. We only need to check the lower bound now, we
    // did the upper bound by clipping the angular speed.
    angle_at_observation = kalman_angle + direction * max(0, 
        direction * signed_angle(trigger_angle - direction * sector_transition_confidence - kalman_angle));

    // Calculate the new angle variance by combining the trigger variance and the estimated angle variance.
    angle_variance_at_observation = clip_to(2, sector_variance,
        trigger_variance * estimated_distance_variance / (trigger_variance + estimated_distance_variance));
    
    // Update the angular speed using the clipped speed and the adjustment calculated with the kalman gain.
    angular_speed_at_observation = clipped_angular_speed + speed_adjustment;

    // Calculate the new angular speed variance by the old variance and the estimated speed error variance.
    angular_speed_variance_at_observation = clip_to(2, initial_angular_speed_variance,
        angular_speed_variance * estimated_speed_error_variance / (angular_speed_variance + estimated_speed_error_variance));

    // Flag to the PWM cycle loop that we have a new observation.
    new_observation = true;
    angle_valid = true;
}


static inline void pwm_cycle_and_adc_update(){
    // Reserve the first 3 bits for the hall sensors.
    latest_readout.readout_number = adc_update_number;
        
        
    // U and W phases are measured at the same time, followed by V and the reference voltage.
    // Each sampling time is 20cycles, and the conversion time is 12.5 cycles. At 12MHz this is
    // 2.08us. The injected sequence is triggered by TIM1 channel 4, which is set to trigger
    // 16ticks after the update event (PWM counter resets). This is a delay of 16/72MHz = 222ns.
    
    // For reference a PWM period is 1536 ticks, so the PWM frequency is 72MHz / 1536 / 2 = 23.4KHz.
    // The PWM period lasts 1/23.4KHz = 42.7us.
    
    latest_readout.u_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    // Note: in the v0 board the V phase shunt is connected in reverse to the current sense amplifier.
    latest_readout.v_readout = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    
    latest_readout.w_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
    latest_readout.ref_readout = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
    
    latest_readout.pwm_commands = get_combined_motor_pwm_duty();

    if(new_observation) {
        new_observation = false;
        time_since_observation = 0;
    }

    time_since_observation = min(time_since_observation + time_increment_per_cycle, max_time_between_observations);


    
    const int estimated_angle = normalize_angle(angle_at_observation + angular_speed_at_observation * time_since_observation / scale);

    // Scale down the angle to 8 bits so we can use a lookup table for the voltage targets.
    const uint8_t angle = estimated_angle * 256 / max_angle_unit;
    
    latest_readout.position = angle | (hall_state << 13) | angle_valid << 12;

    adc_update_number += 1;
    
    update_motor_control_registers(angle);
    
    
    // Always write the readout to the history buffer.
    if(xQueueSendToBackFromISR(readouts_queue, &latest_readout, NULL) != pdPASS){
        if (readouts_allow_missing) {
            // We didn't have space to add the latest readout. Discard the oldest readout and try again; this time 
            // it must work or we have a bigger error.
            readouts_missed += 1;
            if (readouts_to_send > static_cast<int>(HISTORY_SIZE)) readouts_to_send -= 1;

        } else {
            // If we filled the queue without overwriting, we now need to send the data over USB.
            // When the data is sent, overwriting will be re-enabled.
            readouts_allow_sending = true;
        }
    }
}