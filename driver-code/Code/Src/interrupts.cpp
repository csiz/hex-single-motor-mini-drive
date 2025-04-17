#include "interrupts.hpp"

#include "io.hpp"
#include "data.hpp"
#include "constants.hpp"
#include "motor_control.hpp"
#include "main.h" // For the Error_Handler function.


#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_ll_gpio.h>


// Try really hard to keep interrupts fast. Use short inline functions that only rely 
// on chip primitives; don't use division, multiplication, or floating point operations.



// Hall sensors
// ------------

// Hall states as bits, 0b001 = hall 1, 0b010 = hall 2, 0b100 = hall 3.
uint8_t hall_state = 0b000;
// The 6 valid configurations of hall sensors in trigonometric order.
uint8_t hall_sector = 0;
// Whether the sector is valid; it will be false if the magnet is not present or the sensors are not ready.
bool hall_sector_valid = false;


// Position tracking
// -----------------

// Whether the position is valid.
bool angle_valid = false;
// We need to keep track of the previous hall sector to determine the direction of rotation.
uint8_t previous_hall_sector = 0;
// Whether the previous hall sector was valid; initialize anew when the magnet is placed back.
bool previous_hall_sector_valid = false;

// Time, in our units, since the last observation of the hall sensor. The PWM cycle loop updates
// this variable.
int time_since_observation = 0;
// Flag to indicate a new observation; the hall sensor interrupt sets the flag for the PWM cycle loop.
bool new_observation = false;

// Estimated angle at the last observation; this is our best guess of the angle using a kalman filter.
int angle_at_observation = 0;
// Variance of the angle at the last observation.
int angle_variance_at_observation = 0;
// Estimated angular speed at the last observation; this is our best guess of the speed using a kalman filter.
int angular_speed_at_observation = 0;
// Variance of the angular speed at the last observation.
int angular_speed_variance_at_observation = 0;


// Phase Currents
// --------------

float current_u = 0;
float current_v = 0;
float current_w = 0;

// Motor Control
// -------------

uint16_t motor_u_pwm_duty = 0;
uint16_t motor_v_pwm_duty = 0;
uint16_t motor_w_pwm_duty = 0;

// Test Schedule
// -------------

size_t schedule_counter = 0;
size_t schedule_stage = 0;

bool test_procedure_starting = true;



// Constants
// ---------

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


// Note:
// Speed and acceleration are written in degrees per ms and per ms^2 respectively.

// Initial speed estimate.
const int initial_angular_speed = 0;
// Start with a high speed variance.
const int initial_angular_speed_variance = square(scale * max_angle_unit * 30 / 360 / time_units_per_millisecond);

// Precalculate the acceleration variance divided by 4. Note the scaled is squared twice.
const int angular_acceleration_variance_div_4 = square(square_scale * max_angle_unit / 1 / 50 / time_units_per_millisecond / time_units_per_millisecond) / 4;

// Maximum distance to a trigger angle. Don't let the estimated angle deviate
// from the hall sensor angle by more than this value to keep the estimate
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



// Motor Currents
// --------------



// Voltage reference for the ADC; it's a filtered 3.3V that powers the board.
const float adc_voltage_reference = 3.3;
// Shunt resistance for the motor phase current sensing is 10mOhm, 500mW resistor.
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


// Motor control
// -------------

// Motor voltage fraction for the 6-step commutation.
const uint16_t motor_voltage_table_pos[6][3] = {
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
};

// Surpirsingly good schedule for the 6-step commutation.
const uint16_t motor_voltage_table_neg[6][3] {
    {0,        0,        PWM_BASE},
    {PWM_BASE, 0,        PWM_BASE},
    {PWM_BASE, 0,        0       },
    {PWM_BASE, PWM_BASE, 0       },
    {0,        PWM_BASE, 0       },
    {0,        PWM_BASE, PWM_BASE},
};

const uint16_t phases_waveform[256] = {
	1330, 1349, 1366, 1383, 1399, 1414, 1429, 1442, 1455, 1466, 1477, 1487, 1496, 1504, 1511, 1518,
	1523, 1527, 1531, 1534, 1535, 1536, 1536, 1535, 1533, 1530, 1526, 1521, 1516, 1509, 1501, 1493,
	1484, 1474, 1462, 1450, 1438, 1424, 1409, 1394, 1378, 1361, 1343, 1324, 1304, 1284, 1263, 1241,
	1219, 1195, 1171, 1147, 1121, 1095, 1068, 1041, 1013,  984,  955,  925,  895,  864,  832,  800,
	 768,  735,  702,  668,  634,  599,  565,  529,  494,  458,  422,  385,  349,  312,  275,  238,
	 200,  163,  126,   88,   50,   13,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   13,   50,   88,  126,  163,
	 200,  238,  275,  312,  349,  385,  422,  458,  494,  529,  565,  599,  634,  668,  702,  735,
	 768,  800,  832,  864,  895,  925,  955,  984, 1013, 1041, 1068, 1095, 1121, 1147, 1171, 1195,
	1219, 1241, 1263, 1284, 1304, 1324, 1343, 1361, 1378, 1394, 1409, 1424, 1438, 1450, 1462, 1474,
	1484, 1493, 1501, 1509, 1516, 1521, 1526, 1530, 1533, 1535, 1536, 1536, 1535, 1534, 1531, 1527,
	1523, 1518, 1511, 1504, 1496, 1487, 1477, 1466, 1455, 1442, 1429, 1414, 1399, 1383, 1366, 1349
};




// Functions
// ---------


// Compute motor phase currents using latest ADC readouts. 
void calculate_motor_phase_currents_gated(){
    // TODO: rework
    return;

    // Get the latest readout; we have to gate the ADC interrupt so we copy a consistent readout.
    NVIC_DisableIRQ(ADC1_2_IRQn);
    const UpdateReadout readout = latest_readout;
    NVIC_EnableIRQ(ADC1_2_IRQn);

    const int32_t readout_diff_u = readout.u_readout - readout.ref_readout;
    const int32_t readout_diff_v = readout.v_readout - readout.ref_readout;
    const int32_t readout_diff_w = readout.w_readout - readout.ref_readout;

    // The amplifier voltage output is specified by the formula:
    //     Vout = (Iload * Rsense * GAIN) + Vref
    // Therefore:
    //     Iload = (Vout - Vref) / (Rsense * GAIN)
    // Where:
    //     Vout = adc_current_readout / adc_max_value * adc_voltage_reference;

    current_u = readout_diff_u * current_conversion;
    current_v = readout_diff_v * current_conversion;
    current_w = readout_diff_w * current_conversion;
}



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

// Initialize the position tracking system. 
// 
// This should be called once at startup to get the initial hall sensor state.
void initialize_position_tracking(){
    update_position_observation();
}



static inline void update_timeout(){
    if (duration_till_timeout > 0) {
        duration_till_timeout -= 1;
    } else {
        motor_break();
    }
}

static inline void sector_motor_control(){
    // Update motor control registers only if actively driving.

    const uint16_t (*motor_voltage_table)[3] = 
        driver_state == DriverState::DRIVE_POS ? motor_voltage_table_pos : 
        driver_state == DriverState::DRIVE_NEG ? motor_voltage_table_neg : 
        nullptr;
    if (motor_voltage_table == nullptr) return;

    // Note: The registers need to be left unchanged whilst running in the calibration modes.

    // Use the hall sensor state to determine the motor position and commutation.
    if (not hall_sector_valid) {
        motor_break();
        return;
    }

    const uint16_t voltage_phase_u = motor_voltage_table[hall_sector][0];
    const uint16_t voltage_phase_v = motor_voltage_table[hall_sector][1];
    const uint16_t voltage_phase_w = motor_voltage_table[hall_sector][2];


    motor_u_pwm_duty = voltage_phase_u * pwm_command / PWM_BASE;
    motor_v_pwm_duty = voltage_phase_v * pwm_command / PWM_BASE;
    motor_w_pwm_duty = voltage_phase_w * pwm_command / PWM_BASE;
}

static inline void smooth_motor_control(uint8_t angle){
    const int direction = 
        driver_state == DriverState::DRIVE_SMOOTH_POS ? +1 : 
        driver_state == DriverState::DRIVE_SMOOTH_NEG ? -1 : 
        0;
    if (direction == 0) return;

    if (not angle_valid) {
        motor_break();
        return;
    }


    const int target_angle = (256 + static_cast<int>(angle) + direction * static_cast<int>(leading_angle)) % 256;

    const uint16_t voltage_phase_u = phases_waveform[target_angle];
    const uint16_t voltage_phase_v = phases_waveform[(256 + target_angle - 85) % 256];
    const uint16_t voltage_phase_w = phases_waveform[(256 + target_angle - 170) % 256];

    motor_u_pwm_duty = voltage_phase_u * pwm_command / PWM_BASE;
    motor_v_pwm_duty = voltage_phase_v * pwm_command / PWM_BASE;
    motor_w_pwm_duty = voltage_phase_w * pwm_command / PWM_BASE;
}

static inline void test_motor_control(){
    if(active_schedule == nullptr) return motor_break();

    if (test_procedure_starting) {
        test_procedure_starting = false;
        schedule_counter = 0;
        schedule_stage = 0;
            
        readouts_to_send = 0;
        readouts_allow_sending = false;
        readouts_allow_missing = false;

        // Clear the readouts buffer.
        UpdateReadout discard_readout = {};
        while(xQueueReceiveFromISR(readouts_queue, &discard_readout, NULL) == pdPASS) /* Discard all readouts up to test start. */;
        // Store exactly HISTORY_SIZE readouts; until we fully transmit the test.

        enable_motor_outputs();
    }

    const PWMSchedule & schedule = *active_schedule;


    motor_u_pwm_duty = schedule[schedule_stage].u;
    motor_v_pwm_duty = schedule[schedule_stage].v;
    motor_w_pwm_duty = schedule[schedule_stage].w;

    // Go to the next step in the schedule.
    schedule_counter += 1;
    if (schedule_counter >= schedule[schedule_stage].duration) {
        schedule_stage += 1;
        schedule_counter = 0;
    }
    // Next stage, unless we're at the end of the schedule.
    if (schedule_stage >= SCHEDULE_SIZE) {
        
        motor_break();
        readouts_to_send = HISTORY_SIZE;
        readouts_allow_sending = true;
        // Reset the first run flag.
        test_procedure_starting = true;
        // Reset active schedule so we can start another test.
        active_schedule = nullptr;
    }
}

// Critical function!! 23KHz PWM cycle
// -----------------------------------

static inline void pwm_cycle_and_adc_update(){
    // Write the current readout index.
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
    
    
    
    // Check if we had a new hall sensor observation.
    if(new_observation) {
        new_observation = false;
        time_since_observation = 0;
    }
    // Increment the time since the last observation.
    time_since_observation = min(time_since_observation + time_increment_per_cycle, max_time_between_observations);
    
    const int estimated_angle = normalize_angle(angle_at_observation + angular_speed_at_observation * time_since_observation / scale);
    
    
    // Scale down the angle to 8 bits so we can use a lookup table for the voltage targets.
    const uint8_t angle = estimated_angle * 256 / max_angle_unit;
    
    latest_readout.position = angle | (hall_state << 13) | angle_valid << 12;
    
    // Write the previous pwm duty cycle to this readout, it should have been active during the prior to the ADC sampling.
    latest_readout.pwm_commands = motor_u_pwm_duty * PWM_BASE * PWM_BASE + motor_v_pwm_duty * PWM_BASE + motor_w_pwm_duty;
    
    // Always try to write the readout to the history buffer.
    if(xQueueSendToBackFromISR(readouts_queue, &latest_readout, NULL) == errQUEUE_FULL){
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

    // Update motor control.
    switch (driver_state) {
        case DriverState::OFF:
            // Short all motor phases to ground.
            motor_u_pwm_duty = 0;
            motor_v_pwm_duty = 0;
            motor_w_pwm_duty = 0;
            
            // Immediately update and enable the motor outputs.
            enable_motor_outputs();
            break;
            
        case DriverState::FREEWHEEL:
            // Reset PWM duty cycle to 0 anyway.
            motor_u_pwm_duty = 0;
            motor_v_pwm_duty = 0;
            motor_w_pwm_duty = 0;
            
            // Set all motor phases to floating voltage/tristate.
            disable_motor_outputs();
            break;
    
        case DriverState::TEST_SCHEDULE:
            // Quickly update the PWM settings from the test schedule.
            test_motor_control();
            break;
            
        case DriverState::DRIVE_NEG:
        case DriverState::DRIVE_POS:
            sector_motor_control();
            enable_motor_outputs();
            update_timeout();
            break;
        case DriverState::DRIVE_SMOOTH_POS:
        case DriverState::DRIVE_SMOOTH_NEG:
            smooth_motor_control(angle);
            enable_motor_outputs();
            update_timeout();
            break;
        case DriverState::HOLD:
            // Set the duty cycle and hold.
            motor_u_pwm_duty = hold_u_pwm_duty;
            motor_v_pwm_duty = hold_v_pwm_duty;
            motor_w_pwm_duty = hold_w_pwm_duty;
            enable_motor_outputs();
            update_timeout();
            break;
    }

    set_motor_u_pwm_duty_cycle(motor_u_pwm_duty);
    set_motor_v_pwm_duty_cycle(motor_v_pwm_duty);
    set_motor_w_pwm_duty_cycle(motor_w_pwm_duty);
}


// Interrupt handlers
// ------------------



void adc_interrupt_handler(){
    const bool injected_conversions_complete = LL_ADC_IsActiveFlag_JEOS(ADC1);
    if (injected_conversions_complete) {
        pwm_cycle_and_adc_update();

        adc_update_number += 1;
        LL_ADC_ClearFlag_JEOS(ADC1);
    } else {
        Error_Handler();
    }
}



void dma_interrupt_handler() {

}

// Timer 1 is updated every motor PWM cycle; at ~ 70KHz.
void tim1_update_interrupt_handler(){
    if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
        // Note, this updates on both up and down counting, get direction with: LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
        
        tim1_update_number += 1;

        LL_TIM_ClearFlag_UPDATE(TIM1);
    } else {
        Error_Handler();
    }
}

void tim1_trigger_and_commutation_interrupt_handler() {
    // We shouldn't trigger this, but including for documentation.
    Error_Handler();
}


void tim2_global_handler(){
    // The TIM2 updates at a frequency of about 1KHz. Our motor might rotate slower than this
    // so we have to count updates (overflows) between hall sensor triggers.
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        update_position_unobserved();

        tim2_update_number += 1;
        LL_TIM_ClearFlag_UPDATE(TIM2);

    // The TIM2 channel 1 is triggered by the hall sensor toggles. Use it to measure motor rotation.
    } else if (LL_TIM_IsActiveFlag_CC1(TIM2)) {
        update_position_observation(); 

        tim2_cc1_number += 1;
        LL_TIM_ClearFlag_CC1(TIM2);
    } else {
        Error_Handler();
    }
}