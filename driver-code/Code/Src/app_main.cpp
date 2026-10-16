#include "app_main.hpp"

#include "interrupts.hpp"
#include "io.hpp"
#include "usb_com.hpp"
#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "constants.hpp"
#include "dq0_transform.hpp"

#include <cstring>
#include <memory>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_ll_cordic.h>





// Main loop stats tracking
// ------------------------

// Keep track of how many readouts we see.
uint16_t last_readout_number = 0;

// A number for the main loop iterations.
uint32_t main_loop_number = 0;

// Loop number the last time we counted run time stats.
uint32_t timing_loop_number = 0;

// Readout number the last time we counted run time stats.
uint16_t timing_readout_number = 0;

// Minimum period between timing updates in milliseconds.
const uint32_t min_timing_period_millis = 50;

// Time of the last run time stats calculation.
uint32_t last_update_time_millis = 0;


// Communication buffers and connection state
// ------------------------------------------

const uint32_t partial_message_timeout_ms = 500;

hex_mini_drive::ConsistentOverheadByteStuffing<hex_mini_drive::MAX_MESSAGE_SIZE> encoder = {};

// Time of the last USB packet received; used to detect timeouts.
uint32_t last_receive_time = 0;

// Number of bytes discarded due to invalid messages.
uint32_t bytes_discarded = 0;

// Count the number of bytes sent successfully.
uint32_t bytes_sent = 0;

// Whether we should stream data continuously.
uint32_t stream_state = 0;

// Number of readouts to snapshot and send over USB.
uint32_t readouts_to_send = 0;


// Initialization
// --------------


void app_init() {

  // Setup the CORDIC engine to compute atan2 using 32bit parameters.
  LL_CORDIC_Config(
    CORDIC, 
    LL_CORDIC_FUNCTION_PHASE,     // Calculates phase and magnitude
    LL_CORDIC_PRECISION_6CYCLES,   // Balance of precision vs execution speed
    LL_CORDIC_SCALE_0,            // Scale factor 0 (Input bounds: [-1, 1])
    LL_CORDIC_NBWRITE_2,          // Expecting 2 writes: X then Y
    LL_CORDIC_NBREAD_2,           // Yields 2 reads: Phase then Magnitude
    LL_CORDIC_INSIZE_32BITS,       // 32-bit Q1.31 fixed-point input
    LL_CORDIC_OUTSIZE_32BITS     // 32-bit Q1.31 fixed-point output
  );
  
  io_init();

  set_GREEN_LED(0xFF);

  usb_init();
}



// Communications and commands
// ---------------------------

static inline hex_mini_drive::Version make_version_object() {
  hex_mini_drive::Version result;
  std::strncpy(
    reinterpret_cast<char*>(result.data()), 
    hex_mini_drive::VERSION_STRING, hex_mini_drive::VERSION_STRING_SIZE);
  return result;
}

static inline hex_mini_drive::DriveState get_checked_drive_state(hex_mini_drive::DriveState const& drive_state) {
  return hex_mini_drive::DriveState{
    .motor_flags = drive_state.motor_flags,
    .duration = drive_state.duration,
    .pwm_value = clip_to(-pwm_max, +pwm_max, drive_state.pwm_value * control_parameters.motor_direction),
    .angle_increment = drive_state.angle_increment * control_parameters.motor_direction,
  };
}

// Queue a message to the USB send buffer if there's enough space for it.
static inline bool queue_message(hex_mini_drive::Message const& message) {
  // Check if we have space before pushing the message on the buffer.
  // To speed up the check we will assume we write the maximum sized message.
  uint8_t * write_buffer = usb_reserve_send_pointer(hex_mini_drive::MAX_MESSAGE_SIZE);

  // Not enough space in the USB send buffer; bail out early.
  if (write_buffer == nullptr) return false;

  // Serialize the message as bytes into the encoder's buffer.
  encoder.encoding_buffer_size = hex_mini_drive::write_message(
    encoder.encoding_buffer, 
    hex_mini_drive::MAX_MESSAGE_SIZE, 
    message
  );

  // COBS encode the message and get the actual size used (it will be 0 for errors).
  const size_t write_size = encoder.encode_message(write_buffer, hex_mini_drive::MAX_MESSAGE_SIZE);

  // Tell the USB controller that we've written data to the buffer.
  usb_mark_written(write_size);

  bytes_sent += write_size;

  return true;
}

// Handle the command on the buffer; returns whether there was an error.
static inline void handle_message(hex_mini_drive::Message const& message) {

  using namespace hex_mini_drive;

  switch (message.message_code) {
    case NULL_MESSAGE_CODE:
      // No command received; ignore it.
      return;

    case GET_VERSION: {
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::VERSION,
        .message_data = make_version_object()
      });
      return;
    }

    case STREAM_READOUTS: {
      // Continuously stream data if duration > 0.
      stream_state = std::get<StreamReadouts>(message.message_data).stream_state;
      // We should also stop the snapshot behaviour.
      readouts_to_send = 0;
      return;
    }

    case GET_READOUTS_SNAPSHOT: {
      // Get the number of readouts to snapshot and send over USB.
      readouts_to_send = min(
        hex_mini_drive::HISTORY_SIZE, 
        std::get<StreamReadouts>(message.message_data).stream_state
      );
      // We should also stop the continuous streaming behaviour.
      stream_state = false;
      return;
    }

    case GET_INFO: {
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::INFO,
        .message_data = info
      });
      return;
    }

    // Turn off the motor driver.
    case SET_STATE_OFF: {
      while(not drive_state_queue.write_if_available(breaking_drive_state)){
        // Wait until we successfully write the breaking state to the queue.
      };
      return;
    }

    // Freewheel the motor.
    case SET_STATE_FREEWHEEL: {
      while(not drive_state_queue.write_if_available(freewheel_drive_state)){
        // Wait until we successfully write the freewheel state to the queue.
      };
      return;
    }

    case SET_DRIVE_STATE: {
      auto const& drive_state = std::get<DriveState>(message.message_data);
      // Only write the drive_state if there is space available in the queue.
      drive_state_queue.write_if_available(get_checked_drive_state(drive_state));
      return;
    }

    case SET_DRIVE_SCHEDULE: {
      // Only write if there's space for the largest possible schedule size.
      if (drive_state_queue.write_available() < hex_mini_drive::DRIVE_SCHEDULE_SIZE) return;

      size_t i = 0;
      for (; i < hex_mini_drive::DRIVE_SCHEDULE_SIZE; i++) {
        auto const& drive_state = std::get<DriveSchedule>(message.message_data)[i];
        // The schedule lasts until the first state with 0 duration.
        if (drive_state.duration == 0) break;
        drive_state_queue.write_at_offset(i, get_checked_drive_state(drive_state));
      }
      drive_state_queue.mark_written(i);
      
      return;
    }

    // Drive the motor.
    case SET_STATE_DRIVE_6_SECTOR: {
      // set_motor_command(DriveState{ 
      //   .mode = DriverMode::DRIVE_6_SECTOR, 
      //   .duration = std::get<BasicDriveCommand>(message.message_data).duration, 
      //   .active_pwm = std::get<BasicDriveCommand>(message.message_data).pwm_value * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_SMOOTH: {
      // TODO:...
      return;
    }

    case SET_STATE_DRIVE_TORQUE: {
      // set_motor_command(DriveState{
      //   .mode = DriverMode::DRIVE_TORQUE, 
      //   .duration = std::get<SetStateDriveTorque>(message.message_data).duration,
      //   .target = std::get<SetStateDriveTorque>(message.message_data).target_current * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_BATTERY_POWER: {
      // set_motor_command(DriveState{
      //   .mode = DriverMode::DRIVE_BATTERY_POWER, 
      //   .duration = std::get<SetStateDriveBatteryPower>(message.message_data).duration,
      //   .target = std::get<SetStateDriveBatteryPower>(message.message_data).target_power * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_SPEED: {
      // set_motor_command(DriveState{
      //   .mode = DriverMode::DRIVE_SPEED, 
      //   .duration = std::get<SetStateDriveSpeed>(message.message_data).duration,
      //   .target = std::get<SetStateDriveSpeed>(message.message_data).target_speed * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_TORQUE_SPEED: {
      // set_motor_command(DriveState{
      //   .mode = DriverMode::DRIVE_TORQUE_SPEED, 
      //   .duration = std::get<SetStateDriveTorqueSpeed>(message.message_data).duration,
      //   .current_target = std::get<SetStateDriveTorqueSpeed>(message.message_data).target_current * control_parameters.motor_direction,
      //   .speed_target = std::get<SetStateDriveTorqueSpeed>(message.message_data).target_speed * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_SEEK_ANGLE: {
      // set_motor_command(DriveState{
      //   .mode = DriverMode::SEEK_ANGLE, 
      //   .duration = std::get<SetStateSeekAngle>(message.message_data).duration,
      //   .speed_target = std::get<SetStateSeekAngle>(message.message_data).target_speed * control_parameters.motor_direction,
      //   .target = std::get<SetStateSeekAngle>(message.message_data).target_current * control_parameters.motor_direction,
      //   .seek_angle = SeekAngle{
      //     .target_angle = std::get<SetStateSeekAngle>(message.message_data).target_angle * control_parameters.motor_direction,
      //     .target_rotation = std::get<SetStateSeekAngle>(message.message_data).target_rotation * control_parameters.motor_direction,
      //   }
      // });
      return;
    }

    case GET_CALIBRATION:
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION,
        .message_data = calibration
      });
      return;

    case SET_CALIBRATION:
      calibration = std::get<Calibration>(message.message_data);
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION,
        .message_data = calibration
      });
      return;
        
    case RESET_CALIBRATION:
      // Reset the current factors to the default values.
      calibration = default_calibration;
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION,
        .message_data = calibration
      });
      return;

    case GET_CONTROL_PARAMETERS:
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CONTROL_PARAMETERS,
        .message_data = control_parameters
      });
      return;

    case SET_CONTROL_PARAMETERS:
      control_parameters = std::get<ControlParameters>(message.message_data);
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CONTROL_PARAMETERS,
        .message_data = control_parameters
      });
      return;

    case RESET_CONTROL_PARAMETERS:
      control_parameters = default_control_parameters;
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CONTROL_PARAMETERS,
        .message_data = control_parameters
      });
      return;

    case SAVE_SETTINGS_TO_FLASH:
      if(is_motor_safed()){
        save_settings_to_flash(calibration, control_parameters);

        calibration = get_calibration();
        control_parameters = get_control_parameters();
        
        queue_message(hex_mini_drive::Message{
          .message_code = hex_mini_drive::MessageCode::SETTINGS_SAVED_TO_FLASH,
          .message_data = {}
        });
        return;
      }

    // We shouldn't receive these messages; the driver only sends them.
    case VERSION:
    case SETTINGS_SAVED_TO_FLASH:
    case CALIBRATION:
    case CONTROL_PARAMETERS:
    case READOUT:
    case INFO:
    case UNIT_TEST_OUTPUT:
      return;
  }
}

// Keep zeroes to 0, check for nan and inifities and catch those with 0 too. Otherwise cap to minimum value.
static inline float validate_parameter(float const& value, float const& minimum_value) {
  if (value == 0 or not std::isfinite(value)) return 0;
  return max(minimum_value, value);
}

static inline void process_readout(hex_mini_drive::Readout const& readout) {
  // Count if we skip any readouts.
  const uint16_t readouts_since_last_tick = (readout_number_base + readout.readout_number - last_readout_number) % readout_number_base;

  last_readout_number = readout.readout_number;
    
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
      (readout.vcc_voltage < control_parameters.vcc_undervoltage) or
      (readout.resistive_power_average > control_parameters.max_resistive_power) or
      (readout.total_power_average > control_parameters.max_power_draw)
  );

  const float pwm_penalty = (0.1f - 1.f * pwm_too_high) * readouts_since_last_tick;

  info.live_max_pwm = clip_to(
    0, 
    // Leave some pwm headroom for the HFI probing if that is enabled.
    control_parameters.angle_probing_sweep_duration ? pwm_max - control_parameters.angle_probing_pwm : pwm_max, 
    info.live_max_pwm + pwm_penalty
  );

  
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

  const float predicted_emf_voltage = abs_angular_speed * calibration.motor_constant * emf_motor_constant_conversion;

  // Only compute the motor constant if we have a valid angle and the EMF voltage is above the threshold where noise is low.
  const bool compute_motor_constant = readout.angle_fix and (readout.emf_voltage_magnitude > control_parameters.min_emf_for_motor_constant);

  // Get the error (gradient) for the motor constant observer.
  const float motor_constant_error = compute_motor_constant * (readout.emf_voltage_magnitude - predicted_emf_voltage);

  calibration.motor_constant = calibration.motor_constant + motor_constant_error * control_parameters.motor_constant_ki;

  

  // Variable tracking update
  // ------------------------


  // Using DQ0 coordinates again
  //
  // We need to recalculate these values in the main loop as they are not the primary tracking variables.
  
  // Calculate the resistive voltage drop across the coil and MOSFET resistance.
  const float direct_resistive_voltage = readout.direct_current * calibration.resistance;
  const float quadrature_resistive_voltage = readout.quadrature_current * calibration.resistance;

  const float direct_diff = readout.direct_current_diff * current_diff_to_voltage_units;
  const float quadrature_diff = readout.quadrature_current_diff * current_diff_to_voltage_units;

  // The saliency effect is due to the iron between magnets so it occurs twice per 
  // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.
  // For simplicity's sake we keep the saliency_angle_offset as already doubled.
  const int32_t saliency_angle = 2*readout.angle + info.saliency_angle_offset;

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

  const float measured_direct_emf_voltage = direct_inductor_voltage + direct_resistive_voltage - readout.direct_drive_voltage;
  const float measured_quadrature_emf_voltage = quadrature_inductor_voltage + quadrature_resistive_voltage - readout.quadrature_drive_voltage;


  // Get the sin and cos for the emf angle.
  const float cos_emf = get_cos(readout.emf_voltage_angle);
  const float sin_emf = get_sin(readout.emf_voltage_angle);

  // Compute the predicted EMF voltage based on the running estimate at constant emf.
  const float direct_emf_voltage = readout.emf_voltage_magnitude * cos_emf;
  const float quadrature_emf_voltage = readout.emf_voltage_magnitude * sin_emf;


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

  const bool calibration_error_detected = square_voltage_error > control_parameters.voltage_measurement_variance;
  
  // Run calibration when there's error or back EMF is detected.
  if (calibration_error_detected) {

      // Compute the gradients of the loss with respect to each of the tracked parameters.

      // Only update the resistance if we have sufficient current to make the measurement meaningful.
      if(readout.current_magnitude > control_parameters.resistance_current_minimum) {
          const float resistance_gradient = (
              direct_voltage_error * readout.direct_current +
              quadrature_voltage_error * readout.quadrature_current
          ) * voltage_mul_current_to_power;

          
          calibration.resistance = validate_parameter(
            calibration.resistance - resistance_gradient * control_parameters.resistance_ki,
            minimum_resistance
          );
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
          
          // Keep 0 inductance exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
          calibration.inductance = validate_parameter(
            calibration.inductance - inductance_gradient * control_parameters.inductance_ki,
            minimum_inductance
          );

          const float inductance_bias_gradient = (
              direct_voltage_error * direct_inductance_bias_voltage +
              quadrature_voltage_error * quadrature_inductance_bias_voltage
          ) * volts_per_voltage_units_square;
          
          // Keep 0 inductance bias exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
          calibration.inductance_bias = validate_parameter(
            calibration.inductance_bias - inductance_bias_gradient * control_parameters.inductance_bias_ki,
            minimum_inductance_bias
          );

          const float saliency_angle_gradient = (
              direct_voltage_error * (-quadrature_inductance_bias_voltage) +
              quadrature_voltage_error * direct_inductance_bias_voltage
          ) * volts_per_voltage_units_square;

          const float saliency_angle_change = saliency_angle_gradient * control_parameters.saliency_angle_ki;
          info.saliency_angle_offset -= static_cast<int32_t>(saliency_angle_change);
          info.saliency_angle_variance += (square(saliency_angle_change) - info.saliency_angle_variance) * 0.01f;
      }
    // Only track the zero offset while there is no noticeable current being measured.
    } else if (readout.current_magnitude < control_parameters.current_offset_maximum) {
      const auto currents = abc_transform(
        {readout.direct_current, readout.quadrature_current}, 
        readout.zero_current
      );

      // Finally if there's nothing unusual going on, we can update the zero offset for the currents.
      calibration.u_current_zero = clip_to(
          -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
          calibration.u_current_zero + std::get<0>(currents) * control_parameters.zero_current_ki);
      calibration.v_current_zero = clip_to(
          -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
          calibration.v_current_zero + std::get<1>(currents) * control_parameters.zero_current_ki);
      calibration.w_current_zero = clip_to(
          -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
          calibration.w_current_zero + std::get<2>(currents) * control_parameters.zero_current_ki);
  }

  // Accumulate angle adjustments and send over until we reduce the saliency_angle_offset to 0.
  const int32_t pending_adjustment = (info.saliency_angle_offset / 2) * control_parameters.saliency_angle_influence_ki;

  if (set_angle_adjustment(pending_adjustment)) {
    info.saliency_angle_offset -= 2 * pending_adjustment;
  }
}

// Main app loop
// -------------

void app_tick() {
  
  // Run time stats
  // --------------

  // Count loop iterations.
  main_loop_number += 1;

  // Update timing information.
  const uint32_t milliseconds = HAL_GetTick();

  const uint32_t duration_since_timing_update = milliseconds - last_update_time_millis;
  if (duration_since_timing_update > min_timing_period_millis) {
    last_update_time_millis = milliseconds;
    
    float seconds = duration_since_timing_update / 1000.f;

    info.main_loop_rate = (main_loop_number - timing_loop_number) / seconds;
    info.adc_update_rate = ((readout_number_base + last_readout_number - timing_readout_number) % readout_number_base) / seconds;

    timing_loop_number = main_loop_number;
    timing_readout_number = last_readout_number;
  }
  

  // Comms update
  // ------------

  usb_update();

  // It's been too long since we have received anything, discard any partial message.
  if (milliseconds - last_receive_time > partial_message_timeout_ms) {
    encoder.decode_reset();
  }

  size_t rx_size = usb_read_available();
  
  if (rx_size > 0) {
    // We have data to read! Let's read up to the first message, or first error, or store the partial chunk.
    const uint8_t * rx_buffer = usb_get_read_pointer();

    // Read part of the message.
    size_t rx_decoded_size = encoder.decode_chunk(rx_buffer, rx_size);
    // And mark how much we've read to free up space in the usb buffer.
    usb_mark_read(rx_decoded_size);
    
    if (encoder.decode_ready()) {
      // Handle a message if the decoder is ready (we received the appropriately null terminated COBS encoded message).
      hex_mini_drive::Message message;
      if(hex_mini_drive::read_message(message, encoder.decoding_buffer, encoder.decoding_buffer_size)){
        // We have a valid message; handle it.
        handle_message(message);
      } else {
        // Invalid message; ignore it.
        bytes_discarded += rx_decoded_size;
      }
      // We've handled the message, now we can free the encoder for the next message.
      encoder.decode_reset();

    } else if (encoder.decode_empty()){
      
      // The decoder is empty; but we've received data... Either we decoded 0 bytes, in which case the
      // op below doesn't do anything. Or we've decoded some bytes but wrote nothing out because there was an error.
      bytes_discarded += rx_decoded_size;
    }

    // Mark that we've received data at this time; we'll use this to timeout partial messages.
    last_receive_time = milliseconds;
  }


  // Readout post-processing and streaming
  // -------------------------------------

  const size_t readouts_available = readout_history_queue.read_available();

  info.average_buffer_fill_rate += (readouts_available - info.average_buffer_fill_rate) * 0.01f;

  size_t i = 0;
  for (; i < readouts_available; i++) {
    // TODO: Adjust readout
    // --------------
    // Adjust readout to reflect the direction and offset settings from the user.

    auto const& readout = readout_history_queue.peek();

    // First, check whether we need to send; sometimes we need to block in which case we can
    // postpone processing until unblocked.
    const bool need_to_send = stream_state or readouts_to_send;

    const bool queue_success = need_to_send ? queue_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::READOUT,
      .message_data = readout
    }) : false;

    // If we're taking a readout snapshot, use a FIFO strategy for as long as we can
    // to send consecutive readouts to the USB host.
    if (readouts_to_send) {
      if (queue_success) readouts_to_send--;
      else break;
    }

    if (stream_state) {
      // Also queue up info and calibration with each message.
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::INFO,
        .message_data = info
      });
      queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION,
        .message_data = calibration
      });
    }

    // Now that we're done with message queueing we can post-process for before we discard.

    // Post process for calibration, safety limits and advanced control.
    process_readout(readout);

    // All done with this readout, mark it to free up space.
    readout_history_queue.mark_read(1);
  }

  // If the history is full but we're not sending anything then process at least 1 readout to keep the system responsive.
  if (readouts_available == hex_mini_drive::HISTORY_SIZE and i == 0) {
    // Post process for the safety limits, but otherwise skip sending despite the request.
    process_readout(readout_history_queue.peek());

    // Discard the oldest readout, we always want to keep some space for the interrupt
    // loop to write the latest readout for the safety limits update!
    readout_history_queue.mark_read(1);
  }
}



// TODO: Reimplement the advanced motor control algorithms in the main loop.
// -------------------------------------------------------------------------

// // The complete driver state, these values control the motor behaviour.
// struct DriveState {
//     // Settings for the 3 phase MOSFET PWM drivers.
//     MotorOutputs motor_outputs = breaking_motor_outputs;

//     uint16_t motor_enable_flags = enable_flags_all;

//     // Motor driving mode (defaults to short circuit breaking).
//     DriverMode mode;

//     // Duration for the command in pwm cycles.
//     uint16_t duration;

//     // Angle at which the motor is currently driven.
//     int32_t active_angle;
    
//     // The angular speed of the driving wave.
//     float active_speed;

//     // PWM value actively used to drive the motor.
//     float active_pwm;

//     // PWM value that we're aiming for in the tight loop with smooth control.
//     float target_pwm;

//     // The lead angle value used to adjust the angle of the driven phase
//     // to obtain a current that leads the rotor magnetic orientation by 90 degrees.
//     int32_t lead_angle;

//     // Integral term for the PWM control via PID.
//     float pwm_integral;

//     float current_target;
    
//     float speed_target;
    
//     // The target we are aiming for with the command.
//     float target;
    
//     float seek_integral;

//     // The additional data depends on the driver mode.
//     union {
//         // Drive the motor using a fixed PWM schedule.
//         DriveSchedule schedule;

//         // Drive the motor to a specific angle.
//         SeekAngle seek_angle;

//         // Run a test chirp to calibrate/test the motor.
//         TestParameters test_parameters;
//     };
// };



// // Update the motor outputs using simple 6 sector driving based on the hall sensors.
// static inline MotorOutputs update_motor_6_sector(
//     DriveState const& drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){

//     // Update the sector variable.
//     const uint8_t hall_sector = get_hall_sector(readout.state_flags & hall_state_bit_mask);

//     // Check if the magnet is present.
//     const bool angle_valid = hall_sector < hall_sector_base;

//     if (not angle_valid) return breaking_motor_outputs;

//     auto const& motor_sector_driving_table = drive_state.active_pwm >= 0 ? 
//         motor_sector_driving_positive : 
//         motor_sector_driving_negative;

//     // Get the voltage for the three phases from the table.

//     const float voltage_phase_u = motor_sector_driving_table[hall_sector][0];
//     const float voltage_phase_v = motor_sector_driving_table[hall_sector][1];
//     const float voltage_phase_w = motor_sector_driving_table[hall_sector][2];

//     const float abs_pwm = min(
//         readout.live_max_pwm,
//         faster_abs(drive_state.active_pwm)
//     );

//     return MotorOutputs{
//         .u_duty = static_cast<uint16_t>(voltage_phase_u * abs_pwm),
//         .v_duty = static_cast<uint16_t>(voltage_phase_v * abs_pwm),
//         .w_duty = static_cast<uint16_t>(voltage_phase_w * abs_pwm)
//     };
// }

// // Drive the motor using FOC targeting a PWM value. The current is controlled to be as 
// // close to 90 degrees ahead of the magnetic angle as possible; stray currents absorbed.
// // The PWM duty cycle, drive angle, and angle speed are varied smoothly.
// static inline MotorOutputs update_motor_smooth(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // Check if we have an accurate readout angle.
//     const bool current_detected = readout.state_flags & current_detected_bit_mask;

//     drive_state.active_pwm += clip_to(
//         -control_parameters.max_pwm_change,
//         +control_parameters.max_pwm_change,
//         drive_state.target_pwm - drive_state.active_pwm
//     );

//     // Base the direction on the sign of the target PWM.
//     const int32_t active_pwm_sign = sign(drive_state.active_pwm);

//     // Drive towards the ideal angle; however decay to 0 at low EMF voltage.
//     // Ideally the inductor current is exactly 90 degrees ahead of the magnetic angle.
//     // 
//     // Of course, the inductors take a while to charge and the rotor is producing an EMF
//     // which all interacts with the current. However, the current that we end up measuring
//     // should be as close to the 90 degrees as possible for maximum torque per current use.
//     const int32_t ideal_angle = readout.angle + quarter_circle;

//     // TODO: recalculate this based directly on the current_angle_offset and with a parametrized target angle instead of the fixed quarter circle.
//     const int32_t current_angle = readout.angle + readout.current_angle_offset;
//     // Get the error between the measured current and the ideal current angle.
//     const int32_t lead_angle_error = current_detected * (ideal_angle - current_angle);

//     // Adjust the lead angle to keep the current orthogonal to the rotor magnetic pole.
//     drive_state.lead_angle += control_parameters.lead_angle_control_ki * lead_angle_error;

//     // Drive the motor to produce current perpendicular to the magnetic angle.
//     drive_state.active_angle = ideal_angle + active_pwm_sign * drive_state.lead_angle;

//     return update_motor_at_angle(drive_state, readout);
// }
// // Drive the motor with the desired output current (note the DQ0 current is 3/2 phase current).
// // This mode also uses the smooth driving algorithm to drive using field oriented control, thus
// // keeping the current as close to 90 degrees ahead of the magnetic angle as possible. Allowing
// // control of the torque produced by the motor which will be proportional to the current target.
// static inline MotorOutputs update_motor_torque(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
    
//     // Squash very low currents to 0 to avoid noise.
//     const bool current_detected = readout.state_flags & current_detected_bit_mask;
    
//     // Get the signed current magnitude to compare against the target.
//     const float measured_current = current_detected * readout.current_magnitude * get_sin(readout.current_angle_offset);

//     // Calculate the difference between the target and measured current.
//     const float control_error = (drive_state.target - measured_current) * max_drive_current_inverse;

//     // Update the PID control for the torque.
//     drive_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         drive_state.active_pwm + control_error * control_parameters.torque_control_ki
//     );

//     return update_motor_smooth(drive_state, readout);
// }

// // Drive motor using up to a target battery power consumption.
// // 
// // The sign of the target power determines the direction of driving.
// // 
// // This mode will also control power absorption during breaking.
// static inline MotorOutputs update_motor_battery_power(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){

//     const float measured_power = sign(readout.angular_speed) * readout.total_power;

//     const float control_error = (drive_state.target - measured_power) * max_drive_power_inverse;

//     // Update the PID control for the torque.
//     drive_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         drive_state.active_pwm + control_error * control_parameters.battery_power_control_ki
//     );

//     return update_motor_smooth(drive_state, readout);
// }


// static inline MotorOutputs update_motor_speed(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // Calculate the difference between the target and measured current.
//     const float control_error = (drive_state.target - readout.angular_speed) * max_angular_speed_inverse;

//     // Update the PID control for the torque.
//     drive_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         drive_state.active_pwm + control_error * control_parameters.speed_control_ki
//     );

//     return update_motor_smooth(drive_state, readout);
// }

// static inline MotorOutputs update_motor_torque_speed(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     const float command_sign = sign(drive_state.speed_target);
    
//     // Squash very low currents to 0 to avoid noise.
//     const bool current_detected = readout.state_flags & current_detected_bit_mask;
    
//     // Get the signed current magnitude to compare against the target.
//     const float measured_current = current_detected * readout.current_magnitude * get_sin(readout.current_angle_offset);
    
//     // Calculate the difference between the target and measured current.
//     const float current_error = (drive_state.current_target - measured_current) * 
//         max_drive_current_inverse * control_parameters.torque_control_ki;

//     // Calculate the difference between the target and measured current.
//     const float speed_error = (drive_state.speed_target - readout.angular_speed) * 
//         max_angular_speed_inverse * control_parameters.speed_control_ki;


//     const float control_error = (
//         sign(current_error) != command_sign ? current_error :
//         sign(speed_error) != command_sign ? speed_error :
//         current_error + speed_error
//     );

//     // Update the PID control for the torque.
//     drive_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         drive_state.active_pwm + control_error
//     );

//     return update_motor_smooth(drive_state, readout);
// }


// static inline MotorOutputs update_motor_seek_angle(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // The derivative of the error is negative the angular speed.
//     const float position_error_derivative = -readout.angular_speed;
    
//     // Get the error between the target angle and the current angle; predicted to
//     // a future position determined by prediction parameter.
//     const float rotations_error = drive_state.seek_angle.target_rotation - readout.rotations;

//     // Cap to the maximum seek error which is set to 2x the maximum control output so
//     // that the position term can overcome the derivative term. The cap is applied
//     // here so we can use the position_error for the integral term as well.
//     const float position_error = clip_to(
//         -max_seek_position_control, 
//         +max_seek_position_control, 
//         rotations_error * max_seek_rotations_error_inverse
//     );

//     // Proportional term with respect to the maximum position error (control maxes out at greater errors).
//     const float proportional = control_parameters.seek_kp * position_error;
        
//     // Derivative term with respect to the reference speed.
//     const float derivative = clip_to(
//         -max_seek_derivative_control, 
//         +max_seek_derivative_control,
//         control_parameters.seek_kd * position_error_derivative
//     );

    
//     // Calculate the new integral term using the predicted position error (to minimize oscillations).
//     // 
//     // Note that we don't update the integral term just yet, we will update it if the output isn't saturated.
//     const float integral = clip_to(
//         -max_seek_integral_control, 
//         +max_seek_integral_control,
//         drive_state.seek_integral + control_parameters.seek_ki * position_error
//     );

//     // Update the output of the Proportional Integral Derivative (PID) control.
//     float output = (proportional + integral + derivative);
    
//     // Only accumulate the integral if the output isn't saturated.

//     if (output > +1.0) {
//         // Decay the integral if we are saturating the output.
//         drive_state.seek_integral -= drive_state.seek_integral * seek_integral_decay_fraction;
//         // Maximum output.
//         output = +1.0;
//     } else if (output < -1.0) {
//         // Decay the integral if we are saturating the output negatively.
//         drive_state.seek_integral -= drive_state.seek_integral * seek_integral_decay_fraction;
//         // Minimum output.
//         output = -1.0;
//     } else {
//         // Output is not saturated, we can update the integral either direction.
//         drive_state.seek_integral = integral;
//     }

//     drive_state.current_target = output * drive_state.target;
//     drive_state.speed_target = sign(drive_state.current_target) * faster_abs(drive_state.speed_target);

//     return update_motor_torque_speed(drive_state, readout);
// }


// constexpr int16_t hfi_duration = 8;
// constexpr int16_t hfi_skips = (5 /* milliseconds */ * pwm_cycles_per_second / 1000) / hfi_duration;

// static inline MotorOutputs update_motor_hfi_saliency_pulses(
//     DriveState & drive_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // Our puleses will be angled 45 degrees to the saliency angle div 2, the pulses will
//     // each be exactly 2 pwm cycles long, and in this order:
//     // 1. Negative pulse at 45 degrees
//     // 2. Positive pulse at 45 degrees
//     // 3. Positive pulse at -45 degrees
//     // 4. Negative pulse at -45 degrees
//     // So they take 8 pwm cycles.
//     //
//     // We'll also then sleep for n*8 pwm cycles until the next set of pulses.
//     const int32_t angle_from_saliency = readout.angle + readout.saliency_angle_offset / 2;

//     if (readout.readout_number / hfi_duration % hfi_skips == 0) {
//         // Get the step within the pulse sequence.
//         const int32_t step = (readout.readout_number % hfi_duration) / 2;
//         switch (step) {
//             case 0:
//                 drive_state.active_pwm = -drive_state.target;
//                 drive_state.active_angle = angle_from_saliency + quarter_circle/2;
//                 break;
//             case 1:
//                 drive_state.active_pwm = +drive_state.target;
//                 drive_state.active_angle = angle_from_saliency + quarter_circle/2;
//                 break;
//             case 2:
//                 drive_state.active_pwm = +drive_state.target;
//                 drive_state.active_angle = angle_from_saliency - quarter_circle/2;
//                 break;
//             case 3:
//                 drive_state.active_pwm = -drive_state.target;
//                 drive_state.active_angle = angle_from_saliency - quarter_circle/2;
//                 break;
//         }
//     } else {
//         drive_state.active_pwm = 0;
//     }
//     return update_motor_at_angle(drive_state, readout);
// }

