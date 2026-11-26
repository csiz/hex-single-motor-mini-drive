#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <bit>

#include "math_utils.hpp"

#include "hex_mini_drive_interface.hpp"


// Electrical and control limits
// -----------------------------

// 6A max DQ0 driving current.
constexpr float max_drive_current = hex_mini_drive::CURRENT_UNITS_PER_AMP * 6.0;

// 32W max drive power.
constexpr float max_drive_power = 32.0;

// Maximum voltage we can put on the VCC line before any component is out of rating.
constexpr float max_vcc_voltage = hex_mini_drive::VOLTAGE_UNITS_PER_VOLT * 28;

// Expected VCC voltage.
constexpr float nominal_vcc_voltage = hex_mini_drive::VOLTAGE_UNITS_PER_VOLT * 20;

// Angle base is the max uint32_t + 1, so we need to express it as float: std::pow(2, angle_bit_count).
constexpr float angle_base = 4294967296.f;

// Number of MCU clock ticks per PWM cycle; counting up then down.
constexpr int32_t pwm_period = 2 * hex_mini_drive::PWM_BASE;

// Number of PWM cycles per second.
constexpr int32_t pwm_cycles_per_second = hex_mini_drive::CLOCK_FREQUENCY / pwm_period;

constexpr float seconds_per_pwm = 1.f / static_cast<float>(pwm_cycles_per_second);

// We define the maximum drive/measurable electrical rotation frequency as slightly less than
// 12 x the sampling rate. At this speed we move 30degrees around the circle per PWM cycle,
// but not exactly 30degrees so we can sample all angles.
constexpr float max_frequency = pwm_cycles_per_second / 12.0f;

// Number of electrical revolutions per mechanical revolution. This is pole pairs times the number of slot triplets.
constexpr float rotor_revolutions_per_electric = 4;

// Maximum speed that we can drive at our PWM cycle/sampling frequency. In motor rotations per minute (RPM).
constexpr float max_rpm = max_frequency * 60.f / rotor_revolutions_per_electric;

// Maximum angular speed we can achieve in our angle units per PWM cycle.
constexpr float max_angular_speed = angle_base * max_frequency / static_cast<float>(pwm_cycles_per_second);

// Inverse of the maximum angular speed so we can convert to floating point values without doing a division.
constexpr float max_angular_speed_inverse = 1.f / max_angular_speed;

// Square of the maximum angular speed.
constexpr float max_angular_speed_sq = square(max_angular_speed);

// Maximum drive state duration.
constexpr uint16_t max_duration = 0xFFFF;

// Inverse for fast math.
constexpr float max_drive_current_inverse = 1.f / max_drive_current;

// Inverse for fast math.
constexpr float max_drive_power_inverse = 1.f / max_drive_power;


// Unit definitions
// ----------------


// Inverse of the PWM base so we can convert to floating point values without doing a division.
constexpr float pwm_base_inverse = 1.0f / static_cast<float>(hex_mini_drive::PWM_BASE);

// We need to compute and average of the PWM outputs, so this is useful.
constexpr float half_pwm_base_inverse = 0.5f * pwm_base_inverse;

// The ADC has a 12-bit resolution.
constexpr uint16_t adc_max_value = 0xFFF; // 2^12 - 1 == 4095 == 0xFFF.

// Readout number base; we cycle back to 0 instead of reaching this value.
constexpr int32_t readout_number_base = 1 << 16;

// Base and number of hall sectors.
constexpr uint8_t hall_sector_base = 6;

// Number of significant bits for the angle representation (20 bits so 
// we can multiply with 12 bit parameter values and fit within 32 bits).
constexpr size_t angle_bit_count = 32;

// Number of bits in the sin tables; 12 bits gives us 4096 values for a full circle.
constexpr size_t sin_tables_bit_count = 12;

// Precompute the shift value to convert from angle units to sin table units.
constexpr size_t angle_to_sin_table_shift = angle_bit_count - sin_tables_bit_count;


// Inverse of the angle base so we can convert to floating point values without doing a division.
constexpr float angle_base_inverse = 1.0f / angle_base;


// Readout bit packing
// -------------------

constexpr size_t hall_state_bit_offset = 0;
constexpr uint8_t hall_state_bit_mask = 0b111 << hall_state_bit_offset;

constexpr size_t current_detected_bit_offset = 4;
constexpr uint8_t current_detected_bit_mask = 0b1 << current_detected_bit_offset;

constexpr size_t emf_fix_bit_offset = 5;
constexpr uint8_t emf_fix_bit_mask = 0b1 << emf_fix_bit_offset;

constexpr size_t emf_detected_bit_offset = 6;
constexpr uint8_t emf_detected_bit_mask = 0b1 << emf_detected_bit_offset;

// Position constants
// ------------------


// Gear ratio of our chosen motor.
constexpr float gear_ratio = 6.0 * 6.0 * 6.0;

// Total ratio between the electrical angle and the output shaft angle.
constexpr float ratio = rotor_revolutions_per_electric * gear_ratio;


// Electric constants
// ------------------


// Voltage reference for the ADC; it's the in-built reference on the STM32G4.
constexpr float adc_voltage_reference = 2.9f;

constexpr float amplifier_voltage_reference = 3.3f / 2;

// Shunt resistance for the motor phase current sensing is 10mOhm, 500mW resistor.
constexpr float current_shunt_resistance = 0.010f;

// The voltage on the shunt resistor is amplified by INA4181 Bidirectional, Low and 
// High Side Voltage Output, Current-Sense Amplifier.
constexpr float amplifier_gain = 20.0f;


// The formula that determines the current from the ADC readout: 
//   Vout = (Iload * Rsense * GAIN) + Vref
// And the conversion from the ADC readout is given by:
//   Vout = adc_voltage_reference * (adc_current_readout / adc_max_value).
// So the current is:
//   Iload = (Vout - Vref) / (Rsense * GAIN) = adc_voltage_reference * (adc_readout_diff / adc_max_value) / (Rsense * GAIN).
constexpr float adc_to_current_units = hex_mini_drive::CURRENT_UNITS_PER_AMP * adc_voltage_reference / (adc_max_value * current_shunt_resistance * amplifier_gain);

// Maximum current we can measure per phase using our setup. This is less than
// the total adc resolution span because the amplifier is referenced to half 3.3V while
// the adc of the chip is referenced to its internal 2.9V voltage reference. And can
// therefore only measure up to 2.9V. We could measure higher negative currents though.
constexpr float max_measured_current = hex_mini_drive::CURRENT_UNITS_PER_AMP * (adc_voltage_reference - amplifier_voltage_reference) / (current_shunt_resistance * amplifier_gain);



static_assert(max_drive_current < max_measured_current, "We must be able to measure the maximum driving current.");

// We can use 0.86% of the voltage range; we need the inverse constant to compute the required PWM duty to achieve
// a target phase voltage.
// const int32_t pwm_waveform_base = static_cast<int32_t>(hex_mini_drive::PWM_BASE / 0.866);
// const int pwm_for_emf_compensation = -readout.quadrature_emf_voltage * pwm_waveform_base / readout.vcc_voltage;


// Voltage divider between VCC and the ADC reference voltage: 10kohm/110kohm divider
constexpr float vcc_divider = 10.0f/110.0f;

// Conversion factor for the voltage readout from the ADC.
constexpr float voltage_conversion = hex_mini_drive::VOLTAGE_UNITS_PER_VOLT * adc_voltage_reference / (adc_max_value * vcc_divider);

// Current conversion back to amps.
constexpr float amps_per_current_units = 1.0f / hex_mini_drive::CURRENT_UNITS_PER_AMP;

constexpr float amps_per_current_units_square = square(amps_per_current_units);

// Voltage conversion back to volts.
constexpr float volts_per_voltage_units = 1.0f / hex_mini_drive::VOLTAGE_UNITS_PER_VOLT;

// Conversion factor between current and phase resistance voltage.
constexpr float current_to_voltage_units = hex_mini_drive::VOLTAGE_UNITS_PER_VOLT / hex_mini_drive::CURRENT_UNITS_PER_AMP;

// Directly convert voltage * current to power in Watts.
constexpr float voltage_mul_current_to_power = 1.0f / (hex_mini_drive::VOLTAGE_UNITS_PER_VOLT * hex_mini_drive::CURRENT_UNITS_PER_AMP);

// Our dq0 transformation is the power variant form which needs to be corrected by a factor of 3/2.
constexpr float dq0_voltage_mul_current_to_power = voltage_mul_current_to_power * 3.0f / 2.0f;

// Maximum measurable voltage by the ADC given the voltage divider and reference voltage.
constexpr float max_measured_voltage = adc_max_value * voltage_conversion;

// Timing and PWM constants
// ------------------------

// U and V phases are measured at the same time, followed by W and 4th phase (X).
// 
// The phases are preceded by the temperature, voltage and reference readings. The reason
// for this ordering is so the update looph is ready to run as soon as the phases are read.
// The phases must be read at the peak of the PWM cycle while all MOSFETs are shorted to ground.
// 
// Note ADC conversion time is = sample time + 12.5 cycles. The ADC clock is 144MHz / 4.

// Temperature ADC conversion time: 12.5 cycles + 92.5 cycles = 105 cycles = 420 ticks.
constexpr int32_t temperature_sample_time = (6.5f + 12.5f + 1)*4;

// Current ADC conversion time: 12.5 cycles + 6.5 cycles = 19 cycles = 76 ticks.
constexpr int32_t current_sample_time = (6.5f + 12.5f + 1)*4;

// The ADC will read the temperature and reference first then 2 phase currents (for each ADC).
// Try to time the sampling  time of the phase currents symmetrically around the peak of the PWM cycle.
constexpr int32_t sample_lead_time = temperature_sample_time + 0 * current_sample_time;

// The current sense resistors are only connected to the low side MOSFETs. Therefore we can only read
// the phase currents when the low side MOSFETs are on. So we must eat into our PWM cycle to sample
// the currents. The PWM can't be set higher than the ARR - reserve time.
constexpr int32_t sample_reserve_time = 232;

// Auto-reload value for the PWM timer.
constexpr int32_t pwm_autoreload = hex_mini_drive::PWM_BASE - 1;


// Time of a single PWM cycle in seconds.
constexpr float seconds_per_pwm_cycle = 1.0f / static_cast<float>(pwm_cycles_per_second);

// Maximum duty cycle for the high side MOSFETs needs to allow some off time for the
// bootstrap capacitor to charge so it has enough voltage to turn high side MOSFET on.
constexpr int32_t minimum_bootstrap_duty = 30; // 30/144MHz = 208ns

constexpr uint16_t pwm_min = 16;

// Maximum duty cycle for the high side mosfet. We need to allow some off time for the 
// bootstrap capacitor to charge so it has enough voltage to turn mosfet on. And also
// enough time to connect all low side mosfets to ground in order to sample phase currents.
constexpr float pwm_max = hex_mini_drive::PWM_BASE - max(sample_reserve_time, minimum_bootstrap_duty) - static_cast<float>(pwm_min);

// Maximum time (in pwm cycles) while a command is in effect.
constexpr int32_t max_timeout = 0xFFFF;


// Motor control tables
// --------------------

// Motor PWM settings for 6-step commutation in the positive direction (counter-clockwise).
const float motor_sector_driving_positive[6][3] = {
    {0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
};

// Motor PWM settings for 6-step commutation in the negative direction (clockwise).
const float motor_sector_driving_negative[6][3] = {
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0},
};



// Position tracking defaults
// --------------------------

// Define a lot of constants. First, we need to define angles as integers because 
// we can't (shouldn't) use floating point arithmetic in the interrupt handlers.
// Then define the time units. Then speed units, further scaled by a constant to
// have enough precision to represent it.



// Half a circle (pi) aka 180 degrees.
constexpr int32_t half_circle = -angle_base / 2;

constexpr int32_t most_positive_angle = 0x7FFFFFFF;

constexpr int32_t most_negative_angle = 0x80000000;

// 1/3 of a circle (pi/3) aka 120 degrees.
constexpr int32_t third_circle = angle_base / 3;

// 2/3 of a circle (2pi/3) aka 240 degrees.
constexpr int32_t neg_third_circle = -third_circle;

// 1/4 of a circle (pi/4) aka 90 degrees.
constexpr int32_t quarter_circle = angle_base / 4;

// Negative quarter of a circle (-pi/4) aka -90 degrees.
constexpr int32_t neg_quarter_circle = -quarter_circle;

// 1/8 of a circle (pi/8) aka 45 degrees.
constexpr int32_t eight_circle = angle_base / 8;

// Negative 1/8 of a circle (-pi/8) aka -45 degrees.
constexpr int32_t neg_eight_circle = -eight_circle;

// Conversion factor between radians and angle units.
constexpr float radians_to_angle_units = angle_base / (2*3.14159265f);

// Conversion factor between angle units and radians.
constexpr float angle_units_to_radians = 1.f / radians_to_angle_units;

// The angle units per hall sector; 60 degrees.
constexpr int32_t hall_sector_span = angle_base / hall_sector_base;


// Speed and acceleration constants
// --------------------------------

// Note speed values written in degrees per ms and converted to speed units.


constexpr float friction_speed = 0.01f * angle_base / static_cast<float>(pwm_cycles_per_second);

// Conversion factor between our speed units and radians per second.
constexpr float speed_units_to_radians_per_second = angle_units_to_radians * static_cast<float>(pwm_cycles_per_second);

// Conversion factor between radians per second and our speed units.
constexpr float radians_per_second_to_speed_units = 1.f / speed_units_to_radians_per_second;


// Calculation precomputed constants
// ---------------------------------

// Conversion factor between current difference and the implied inductor voltage given the phase inductance (V = L * di/dt).
constexpr float current_diff_to_voltage_units = pwm_cycles_per_second *
    hex_mini_drive::VOLTAGE_UNITS_PER_VOLT / hex_mini_drive::CURRENT_UNITS_PER_AMP;

// Conversion factor to hysteresis effect; hysteresis is Ohm*Henri and we multiply with current and current_diff units and want voltage.
constexpr float hysteresis_conversion = current_diff_to_voltage_units * voltage_mul_current_to_power * hex_mini_drive::VOLTAGE_UNITS_PER_VOLT;

// Conversion factor from speed units to voltage units prefix for motor constant.
constexpr float emf_motor_constant_conversion = hex_mini_drive::VOLTAGE_UNITS_PER_VOLT * speed_units_to_radians_per_second;

constexpr float emf_speed_constant_conversion = radians_per_second_to_speed_units / hex_mini_drive::VOLTAGE_UNITS_PER_VOLT;


constexpr float emf_speed_constant_conversion_inverse = 1.f / emf_speed_constant_conversion;

constexpr float max_speed_constant = 4 * max_angular_speed / (emf_speed_constant_conversion * max_measured_voltage);

constexpr float max_speed_constant_sq = square(max_speed_constant);

constexpr float min_speed_constant = angle_base * 1.f / pwm_cycles_per_second / (emf_speed_constant_conversion * max_measured_voltage);

constexpr float min_speed_constant_sq = square(min_speed_constant);


// Baseline measurement noise
// --------------------------
// 
// The accuracy of our measurements and output variables are limited by hardware. First of all, we read a finite amount of bits,
// for this board it is 12 bit from the ADC, but there also appear to be other random noise. I've noticed that tapping the motor
// produces a noticeable change in readings. It's quite likely some of the noise is coming from actual sound and vibration around
// the motor. There is also electrical noise. Also there's likely some variance in the output of the driving MOSFETs at different
// PWM settings, even if the PWM signal itself is fairly accurate.

// Noise level of the current measurements, in current units.
constexpr float current_measurement_stdev = 8 * adc_to_current_units;

// The difference between current measurements is noisier.
constexpr float current_diff_measurement_stdev = 16 * adc_to_current_units;

// Precomputed conversion for inductance noise. Units of voltage per current (difference) per time step.
constexpr float inductance_measurement_stdev = current_diff_measurement_stdev * current_diff_to_voltage_units;

constexpr float hysteresis_stdev = 16 * current_measurement_stdev * current_diff_measurement_stdev * hysteresis_conversion;

// The noise of our applied voltage is a few units from the maximum we can use.
constexpr float voltage_output_stdev = 4.f * nominal_vcc_voltage * half_pwm_base_inverse;

constexpr float voltage_output_stdev_inverse = 1.0f / voltage_output_stdev;

constexpr float voltage_error_to_angle_units_scaling = radians_to_angle_units * voltage_output_stdev_inverse;

// We also need the squares.
constexpr float default_current_variance = square(current_measurement_stdev);
constexpr float default_inductance_variance = square(inductance_measurement_stdev);
constexpr float default_voltage_variance = square(voltage_output_stdev);
constexpr float default_hysteresis_variance = square(hysteresis_stdev);

// Driver state
// ------------

// Enable all motor outputs (U, V, W).
const uint8_t ENABLE_ALL_OUTPUTS = (
    hex_mini_drive::ENABLE_U_OUTPUT |
    hex_mini_drive::ENABLE_V_OUTPUT |
    hex_mini_drive::ENABLE_W_OUTPUT |
    hex_mini_drive::ENABLE_Z_OUTPUT
);

// Disable all motor outputs.
const uint8_t DISABLE_ALL_OUTPUTS = 0;


// Driver state for safely breaking the motor.
const hex_mini_drive::DriveState breaking_drive_state = {
    .motor_flags = ENABLE_ALL_OUTPUTS,
};

// Driver state for electrically disconnecting the motor.
const hex_mini_drive::DriveState freewheel_drive_state = {
    .motor_flags = DISABLE_ALL_OUTPUTS,
};

// Calibration and Control Parameters
// ----------------------------------

constexpr float minimum_resistance = 0.005f; // This is lower than the sense resistors, something definitely wrong if we reach here.
constexpr float minimum_inductance = 0.000'001f;
constexpr float minimum_inductance_saliency = 0.000'001'0f;
constexpr float minimum_hysteresis_loss = 0.000'000'001f;

// Default to a the planetary 3 phase motor.
const hex_mini_drive::Calibration default_calibration = {
    .u_current_zero = 0.0f,
    .v_current_zero = 0.0f,
    .w_current_zero = 0.0f,
    // Underestimate resistance.
    .resistance = minimum_resistance,
    // Underestimate inductance.
    .inductance = minimum_inductance,
    .inductance_saliency = minimum_inductance_saliency,
    .hysteresis_loss = minimum_hysteresis_loss,
    .eddy_resistance = 0.f,
};

const hex_mini_drive::CalibrationParameters default_calibration_parameters = {
    .current_variance = default_current_variance,
    .inductance_variance = default_inductance_variance,
    .voltage_variance = default_voltage_variance,
    .hysteresis_variance = default_hysteresis_variance,
    .multiplier_stable_threshold = 0.001f,
    .multiplier_change = std::pow(2, -10),
    .multiplier_cap_decay = std::pow(2, -14),
    .excitation_threshold = 0.2f,
    .current_offset_maximum = 16 * adc_to_current_units,
    .zero_current_ki = std::pow(2, -16),
    .resistance_ki = std::pow(2, -10),
    .inductance_ki = std::pow(2, -11),
    .inductance_saliency_ki = std::pow(2, -11),
    .saliency_angle_ki = std::pow(2, -7),
    .hysteresis_loss_ki = std::pow(2, -15),
    .hysteresis_angle_ki = std::pow(2, -9),
    .eddy_resistance_ki = 0,
    .motor_constant_ki = std::pow(2, -10),
    .stats_ki = std::pow(2, -8),
    .instance_duration = max_duration,
    .remember_intermediate = true,
    .advance_when_stable = true,
    .target_pwm_base = 64.f,
    .target_pwm_increment = 16.f,
    .target_pwm_max = 196.f,
    .speed_base = 4.f * 110.f * angle_base / static_cast<float>(pwm_cycles_per_second),
    .speed_multiplier = std::pow(2, 1.f / 12.f),
    .speed_max = angle_base / 12.f,
    .nominal_speed_inverse = 1.0f / (4.f * 110.f * angle_base / static_cast<float>(pwm_cycles_per_second)),
};

// The default control parameters should be set to reasonable values for any motor.
// 
// The reset button will reload these values.
const hex_mini_drive::ControlParameters default_control_parameters = {
    .rotation_zero = 0,
    .angle_zero = 0,
    // Option to flip the motor direction.
    .motor_direction = +1,
    .angle_probing_interval = 0.010f * pwm_cycles_per_second,
    .angle_probing_pwm = 48.0f,
    .angle_probing_sweep_duration = 48,
    .angle_probing_angular_speed = angle_base / 12,

    .vcc_undervoltage = 8.0f * hex_mini_drive::VOLTAGE_UNITS_PER_VOLT,
    .max_resistive_power = 16.0f,
    .max_power_draw = max_drive_power,
    .resistive_power_ki = std::pow(2, -12),
    .power_draw_ki = std::pow(2, -10),

    // Rotor control gains.
    .angle_from_emf_ki = std::pow(2, -4),
    // Gain for the angular speed, it should be much lower than the angle gain to average out the noise.
    .loop_fast_ki = std::pow(2, -3),
    .loop_slow_ki = std::pow(2, -6),
    // Gain for angle adjustments from hall sensors.
    .angle_from_hall_ki = std::pow(2, -4),
    .lead_angle_control_ki = std::pow(2, -8),
    .angle_from_saliency_ki = std::pow(2, -4),


    .emf_min_angular_speed = 25.f * angle_base / static_cast<float>(pwm_cycles_per_second),
    .emf_uncertainty_threshold = 256,

    .torque_control_ki = 2.0f * amps_per_current_units,
    .torque_control_kff = 0.1f,

    .battery_power_control_ki = 4.0f,
    .battery_power_control_kff = 0.1f,
    .speed_control_ki = 100.0f * angle_base_inverse,
    .speed_control_kff = 0.1f,


    .seek_kff = 0.0f,
    .seek_ki = std::pow(2, -16),
    .seek_kp = 1.0f,
    .seek_kd = 0.0f,

    .emf_speed_constant_ki = std::pow(2, -8),
    .emf_angle_suspected_threshold = square(quarter_circle / 3),
    .emf_speed_constant_confirmed_threshold = 16.0f,

};