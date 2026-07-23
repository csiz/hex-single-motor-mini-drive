#include "app_main.hpp"

#include "interrupts.hpp"
#include "interrupts.hpp"
#include "comms.hpp"
#include "io.hpp"

#include <stm32g4xx_hal.h>


uint32_t main_loop_number = 0;
uint32_t last_loop_number = 0;
uint16_t last_readout_number = 0;


const uint32_t min_timing_period_millis = 50;
uint32_t last_update_time_millis = 0;

float main_loop_rate = 0.0f;
float adc_update_rate = 0.0f;


int32_t resistive_power_observer = 0;

int32_t total_power_observer = 0;


void app_init() {

    io_init();

    set_GREEN_LED(0xFF);

    // Get initial hall sensor state and initialize position tracking.
    initialize_angle_tracking();

    comms_init();
}


void app_tick() {
    main_loop_number += 1;

    hex_mini_drive::FullReadout readout = get_readout();

    // Timing
    // ------

    // Update timing information.
    const uint32_t milliseconds = HAL_GetTick();

    const uint32_t duration_since_timing_update = milliseconds - last_update_time_millis;
    if (duration_since_timing_update > min_timing_period_millis) {
        last_update_time_millis = milliseconds;
        
        float seconds = duration_since_timing_update / 1000.f;

        main_loop_rate = (main_loop_number - last_loop_number) / seconds;
        adc_update_rate = ((readout_number_base + readout.readout_number - last_readout_number) % readout_number_base) / seconds;

        last_loop_number = main_loop_number;
        last_readout_number = readout.readout_number;
    }

    // Limits!
    // -------

    // Calculate slowly varying averages of the resistive power; this represents the energy
    // dissipated in the motor coils which should be proportional to the temperature rise.
    const int avg_resistive_power = resistive_power_observer / hires_fixed_point;
    
    // Update the higher resolution observer.
    resistive_power_observer += (readout.resistive_power - avg_resistive_power) * control_parameters.resistive_power_ki;

    // Calculate slowly varying averages of the total power; this represents the energy
    // drawn from the battery. At constant voltage, this is proportional to the current drawn.
    const int avg_total_power = total_power_observer / hires_fixed_point;

    // Update the higher resolution observer.
    total_power_observer += (readout.total_power - avg_total_power) * control_parameters.power_draw_ki;


    // Reduce the maximum output PWM to keep within safe limits:
    // 1. The MOSFET drivers need to be kept in their operating voltage range. Reduce PWM to
    // let the battery recharge our local capacitors.
    // 2. The resistive power heats up the motor coils. Keep it under a threshold to avoid overheating.
    // 3. The total power is a good proxy for total current consumed from the battery.
    // 
    // +limiting_divisor_m1 so we do ceiling of the division.
    // 
    // The penalty should normally be negative indicating we can increase the PWM.
    // TODO: redo penalty calculations.
    const int pwm_penalty = (max(
        vcc_mosfet_driver_undervoltage - readout.vcc_voltage,
        avg_resistive_power - control_parameters.max_resistive_power,
        avg_total_power - control_parameters.max_power_draw,
        faster_abs(readout.angular_speed) - control_parameters.max_angular_speed
    ) + limiting_divisor_m1) / limiting_divisor;

    const int live_max_pwm = clip_to(0, control_parameters.max_pwm, readout.live_max_pwm - pwm_penalty);


    
    // Calculate the motor constant
    // ----------------------------
    // 
    // The motor constant is the ratio of the EMF voltage to the angular speed (in radians per second).
    // 
    // It is also the ratio between the torque produced by the motor and the quadrature current. We 
    // can compute the motor constant from the a spinning motor and use it to estimate our torque.
    // 
    // We calculate the motor constant by gradient descent using the configured integral gain.

    // The voltage magnitude is always positive, also use the positive angular speed.
    const float abs_angular_speed = readout.angular_speed > 0 ? readout.angular_speed : -readout.angular_speed;

    const float predicted_emf_voltage = abs_angular_speed * readout.motor_constant * emf_motor_constant_conversion;

    const bool angle_fix = readout.state_flags & angle_fix_bit_mask;

    // Only compute the motor constant if we have a valid angle and the EMF voltage is above the threshold where noise is low.
    const bool compute_motor_constant = angle_fix and (readout.emf_voltage_magnitude > control_parameters.min_emf_for_motor_constant);

    // Get the error (gradient) for the motor constant observer.
    const float motor_constant_error = compute_motor_constant * (readout.emf_voltage_magnitude - predicted_emf_voltage);

    const float motor_constant = readout.motor_constant + motor_constant_error * control_parameters.motor_constant_ki;


    // Write all values
    // ----------------
    
    readout.main_loop_rate = static_cast<int>(main_loop_rate);
    readout.adc_update_rate = static_cast<int>(adc_update_rate);
    readout.motor_constant = motor_constant;

    // Adjust direction
    // ----------------

    readout.angle = normalize_angle(control_parameters.motor_direction * readout.angle);
    readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
    readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;

    readout.rotations = control_parameters.motor_direction * readout.rotations;
    readout.rotor_acceleration = control_parameters.motor_direction * readout.rotor_acceleration;

    // TODO: this doesn't get reported to the driver loop... we need to rethink that in a bit.
    readout.live_max_pwm = live_max_pwm;
    

    // Comms update
    comms_update(readout);
}