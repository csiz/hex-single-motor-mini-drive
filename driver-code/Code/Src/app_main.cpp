#include "app_main.hpp"

#include "interrupts.hpp"
#include "io.hpp"
#include "usb_com.hpp"
#include "test_schedules.hpp"
#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "constants.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_ll_cordic.h>


// Keep track of how many readouts we see.
uint16_t last_readout_number = 0;

// Main loop stats tracking
// ------------------------
uint32_t main_loop_number = 0;
uint32_t timing_loop_number = 0;
uint16_t timing_readout_number = 0;


const uint32_t min_timing_period_millis = 50;
uint32_t last_update_time_millis = 0;

float main_loop_rate = 0.0f;
float adc_update_rate = 0.0f;



// Calibration tracking
// --------------------

enum struct CalibrationMode {
  NONE,
  RESISTANCE,
  INDUCTANCE,
  ROTATING_CHIRP,
  FIXED_CHIRP
};

CalibrationMode calibration_mode = CalibrationMode::NONE;


// Communication buffers and state
// -------------------------------

const uint32_t partial_message_timeout_ms = 500;
const uint32_t sending_timeout_ms = 500;

hex_mini_drive::ConsistentOverheadByteStuffing<hex_mini_drive::MAX_MESSAGE_SIZE> encoding_buffer = {};

// Time of last USB packet sent; used to detect timeouts.
uint32_t last_send_time = 0;

// Time of the last USB packet received; used to detect timeouts.
uint32_t last_receive_time = 0;

// Number of bytes discarded due to invalid messages.
uint32_t bytes_discarded = 0;

uint32_t bytes_sent = 0;

uint16_t stream_state = 0;
uint16_t stream_last_sent = 0;
size_t readouts_to_send = 0;
size_t readouts_sent = 0;
bool reply_current_calibration = false;
bool reply_control_parameters = false;
bool reply_unit_test = false;
bool reply_saved_to_flash = false;

UnitTestFunction unit_test_function = nullptr;



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


// Start an active test of the motor.
void motor_start_test(PWMSchedule const& schedule, float value, bool take_snapshot) {
  // Clear the readouts buffer of old data.
  readout_history_mark_reset();
  readouts_to_send = take_snapshot ? hex_mini_drive::HISTORY_SIZE : 0;
  readouts_sent = 0;

  // Start the test schedule.
  // TODO:...
  // set_motor_command(DriverState{ 
  //   .mode = DriverMode::SCHEDULE,
  //   .target = value,
  //   .schedule = DriveSchedule{ .pointer = &schedule },
  // });
}

// Run a unit test that takes a function pointer to a test function (which itself takes a buffer); returns whether error occurred.
bool run_unit_test(UnitTestFunction test_function) {
  if (reply_unit_test) return true; // We are already running a unit test.

  // Remember which test to run when the USB queue is ready.
  unit_test_function = test_function;

  // Set the flag to indicate that we have a unit test result ready to send.
  reply_unit_test = true;

  return false;
}


// Handle the command on the buffer; returns whether there was an error.
void handle_message(hex_mini_drive::Message const& message) {

  using namespace hex_mini_drive;

  switch (message.message_code) {
    case NULL_MESSAGE_CODE:
      // No command received; ignore it.
      return;

    case STREAM_FULL_READOUTS: {
      // Continuously stream data if timeout > 0.
      stream_state = std::get<StreamFullReadouts>(message.message_data).stream_state;
      // Also stop the motor if we stop the stream.
      if (not stream_state) set_motor_command(DriverState{.mode = DriverMode::OFF});
      return;
    }
    case GET_READOUTS_SNAPSHOT: {
      // Cancel streaming; so we can take a data snapshot without interruptions.
      stream_state = 0;
      
      // Reply with the full history.
      readout_history_mark_reset();
      readouts_to_send = hex_mini_drive::HISTORY_SIZE;
      readouts_sent = 0;

      return;
    }
    // Turn off the motor driver.
    case SET_STATE_OFF: {
      // Repeat command, with the interrupt guards.
      set_motor_command(DriverState{.mode = DriverMode::OFF});
      return;
    }
        
    // Measure the motor phase currents.
    
    case SET_STATE_TEST_ALL_PERMUTATIONS:
      motor_start_test(
        test_all_permutations, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_GROUND_SHORT:
      motor_start_test(
        test_ground_short, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_POSITIVE_SHORT:
      motor_start_test(
        test_positive_short, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;

    // Drive the motor.
    case SET_STATE_DRIVE_6_SECTOR: {
      set_motor_command(DriverState{ 
        .mode = DriverMode::DRIVE_6_SECTOR, 
        .duration = std::get<BasicDriveCommand>(message.message_data).timeout, 
        .active_pwm = std::get<BasicDriveCommand>(message.message_data).pwm_value * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_PERIODIC: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_PERIODIC, 
        .duration = std::get<SetStateDrivePeriodic>(message.message_data).timeout,
        .active_angle = std::get<SetStateDrivePeriodic>(message.message_data).angle,
        .active_angular_speed = std::get<SetStateDrivePeriodic>(message.message_data).angular_speed * control_parameters.motor_direction,
        .active_pwm = std::get<SetStateDrivePeriodic>(message.message_data).pwm_value,
      });
      return;
    }

    case SET_STATE_DRIVE_SMOOTH: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_SMOOTH, 
        .duration = std::get<SetStateDriveSmooth>(message.message_data).timeout,
        .target_pwm = std::get<SetStateDriveSmooth>(message.message_data).pwm_value * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_TORQUE: {
      // set_motor_command(DriverState{
      //   .mode = DriverMode::DRIVE_TORQUE, 
      //   .duration = std::get<SetStateDriveTorque>(message.message_data).timeout,
      //   .target = std::get<SetStateDriveTorque>(message.message_data).target_current * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_BATTERY_POWER: {
      // set_motor_command(DriverState{
      //   .mode = DriverMode::DRIVE_BATTERY_POWER, 
      //   .duration = std::get<SetStateDriveBatteryPower>(message.message_data).timeout,
      //   .target = std::get<SetStateDriveBatteryPower>(message.message_data).target_power * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_SPEED: {
      // set_motor_command(DriverState{
      //   .mode = DriverMode::DRIVE_SPEED, 
      //   .duration = std::get<SetStateDriveSpeed>(message.message_data).timeout,
      //   .target = std::get<SetStateDriveSpeed>(message.message_data).target_speed * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_DRIVE_TORQUE_SPEED: {
      // set_motor_command(DriverState{
      //   .mode = DriverMode::DRIVE_TORQUE_SPEED, 
      //   .duration = std::get<SetStateDriveTorqueSpeed>(message.message_data).timeout,
      //   .current_target = std::get<SetStateDriveTorqueSpeed>(message.message_data).target_current * control_parameters.motor_direction,
      //   .speed_target = std::get<SetStateDriveTorqueSpeed>(message.message_data).target_speed * control_parameters.motor_direction,
      // });
      return;
    }

    case SET_STATE_SEEK_ANGLE: {
      // set_motor_command(DriverState{
      //   .mode = DriverMode::SEEK_ANGLE, 
      //   .duration = std::get<SetStateSeekAngle>(message.message_data).timeout,
      //   .speed_target = std::get<SetStateSeekAngle>(message.message_data).target_speed * control_parameters.motor_direction,
      //   .target = std::get<SetStateSeekAngle>(message.message_data).target_current * control_parameters.motor_direction,
      //   .seek_angle = SeekAngle{
      //     .target_angle = std::get<SetStateSeekAngle>(message.message_data).target_angle * control_parameters.motor_direction,
      //     .target_rotation = std::get<SetStateSeekAngle>(message.message_data).target_rotation * control_parameters.motor_direction,
      //   }
      // });
      return;
    }

    // Freewheel the motor.
    case SET_STATE_FREEWHEEL:
      set_motor_command(DriverState{ .mode = DriverMode::FREEWHEEL });
      return;

    case SET_CURRENT_CALIBRATION:
      current_calibration = std::get<CurrentCalibration>(message.message_data);
      reset_calibration_variables();
      reply_current_calibration = true;
      return;
        
    case RESET_CURRENT_CALIBRATION:
      // Reset the current factors to the default values.
      current_calibration = default_current_calibration;
      reset_calibration_variables();
      reply_current_calibration = true;
      return;

    case SET_CONTROL_PARAMETERS:
      control_parameters = std::get<ControlParameters>(message.message_data);
      reply_control_parameters = true;
      return;

    case RESET_CONTROL_PARAMETERS:
      control_parameters = default_control_parameters;
      reply_control_parameters = true;
      return;

    case GET_CURRENT_CALIBRATION:
      reply_current_calibration = true;
      return;

    case GET_CONTROL_PARAMETERS:
      reply_control_parameters = true;
      return;

    case SET_ANGLE: {
      set_angle(std::get<SetAngle>(message.message_data).angle);
      return;
    }

    case SET_ROTATIONS: {
      set_rotations(std::get<SetRotations>(message.message_data).rotations);
      return;
    }

    case SAVE_SETTINGS_TO_FLASH:
      if(is_motor_safed()){
        save_settings_to_flash(current_calibration, control_parameters);

        current_calibration = get_current_calibration();
        control_parameters = get_control_parameters();
        
        reply_saved_to_flash = true;
        return;
      }

    // We shouldn't receive these messages; the driver only sends them.
    case SETTINGS_SAVED_TO_FLASH:
    case CURRENT_CALIBRATION:
    case CONTROL_PARAMETERS:
    case READOUT:
    case FULL_READOUT:
    case UNIT_TEST_OUTPUT:
      return;

    
    case SET_STATE_RESISTANCE_CALIBRATION: {
      // calibration_mode = CalibrationMode::RESISTANCE;
      
      // // Clear the readouts buffer of old data.
      // readout_history_mark_reset();
      // readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      // readouts_sent = 0;

      // set_motor_command(DriverState{
      //   .mode = DriverMode::RESISTANCE_CALIBRATION,
      //   .duration = hex_mini_drive::HISTORY_SIZE,
      //   .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      // });
      return;
    }

    case SET_STATE_INDUCTANCE_CALIBRATION: {
      // calibration_mode = CalibrationMode::INDUCTANCE;

      // // Clear the readouts buffer of old data.
      // readout_history_mark_reset();
      // readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      // readouts_sent = 0;
      
      // set_motor_command(DriverState{
      //   .mode = DriverMode::INDUCTANCE_CALIBRATION,
      //   .duration = hex_mini_drive::HISTORY_SIZE,
      //   .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      // });
      return;
    }

    case SET_STATE_ROTATING_CALIBRATION_CHIRP: {
      // calibration_mode = CalibrationMode::ROTATING_CHIRP;
      // // Clear the readouts buffer of old data.
      // readout_history_mark_reset();
      // readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      // readouts_sent = 0;

      // set_motor_command(DriverState{
      //   .mode = DriverMode::ROTATING_CALIBRATION_CHIRP,
      //   .duration = hex_mini_drive::HISTORY_SIZE,
      //   .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      //   .test_parameters = TestParameters{
      //     .test_speed = std::get<TestCommand>(message.message_data).test_speed,
      //     .test_duration = std::get<TestCommand>(message.message_data).test_duration,
      //   },
      // });
      return;
    }
    case SET_STATE_FIXED_CALIBRATION_CHIRP: {
      // calibration_mode = CalibrationMode::FIXED_CHIRP;
      
      // readout_history_mark_reset();
      // readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      // readouts_sent = 0;


      // set_motor_command(DriverState{
      //   .mode = DriverMode::FIXED_CALIBRATION_CHIRP,
      //   .duration = hex_mini_drive::HISTORY_SIZE,
      //   .active_angle = 0,
      //   .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      //   .test_parameters = TestParameters{
      //     .test_speed = std::get<TestCommand>(message.message_data).test_speed,
      //     .test_duration = std::get<TestCommand>(message.message_data).test_duration,
      //     .test_angle = std::get<TestCommand>(message.message_data).test_angle
      //   },
      // });
      return;
    }
    case SET_STATE_HFI_SALIENCY_PULSES: {
      // // Clear the readouts buffer of old data.
      // readout_history_mark_reset();
      // readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      // readouts_sent = 0;
      
      // set_motor_command(DriverState{
      //   .mode = DriverMode::HFI_SALIENCY_PULSES,
      //   .duration = std::get<TestCommand>(message.message_data).test_duration,
      //   .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      // });
      return;
    }
  }
}

void reset_buffers(){
  set_motor_command(DriverState{.mode = DriverMode::OFF});

  encoding_buffer.encode_reset();
  encoding_buffer.decode_reset();
  last_receive_time = 0;
  stream_state = 0;
}


void serialize_message(hex_mini_drive::Message const& message) {
  uint8_t out_buffer[hex_mini_drive::MAX_MESSAGE_SIZE] = {0};
  size_t out_buffer_size = hex_mini_drive::write_message(out_buffer, hex_mini_drive::MAX_MESSAGE_SIZE, message);

  if (out_buffer_size) encoding_buffer.encode_message(out_buffer, out_buffer_size);
}

void queue_response(hex_mini_drive::FullReadout const& readout) {
  // Wait until the previous message has been sent or buffers reset before encoding a new message.
  if (encoding_buffer.is_message_encoded()) return;

  // Queue the readout history to the send buffer.
  if(readouts_to_send > 0){
    const bool reset_ongoing = readout_history_get_reset_flag();
    const bool waiting_for_data = get_readout_history_size() < readouts_to_send;
    
    // Wait until we filled the buffer before sending readouts, to avoid interfering with the ADC loop.
    if (reset_ongoing or waiting_for_data) return;

    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::READOUT,
      .message_data = get_readout_history()[readouts_sent]
    });
    
    // Readout added to the USB buffer.
    readouts_sent += 1;

    if (readouts_sent >= readouts_to_send) {
      // We have sent all the readouts; reset the counters.
      readouts_to_send = 0;
      readouts_sent = 0;
    }

    return;
  }

  // Stream readouts but only once per readout number.
  if(stream_state and stream_last_sent != readout.readout_number){
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::FULL_READOUT,
      .message_data = readout
    });

    stream_last_sent = readout.readout_number;
    return;
  }

  // Send unit test response if requested.
  if (reply_unit_test) {
    hex_mini_drive::Message message = {
      .message_code = hex_mini_drive::MessageCode::UNIT_TEST_OUTPUT,
      .message_data = hex_mini_drive::UnitTestOutput{}
    };
    unit_test_function(
      reinterpret_cast<char*>(std::get<hex_mini_drive::UnitTestOutput>(message.message_data).data()), 
      hex_mini_drive::UNIT_TEST_OUTPUT_SIZE
    );
    serialize_message(message);
    reply_unit_test = false;
    return;
  }

  // Send control parameters if requested.
  if (reply_control_parameters) {
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::CONTROL_PARAMETERS,
      .message_data = control_parameters
    });
    reply_control_parameters = false;
    return;
  }

  // Send current factors if requested.
  if (reply_current_calibration) {
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::CURRENT_CALIBRATION,
      .message_data = current_calibration
    });
    reply_current_calibration = false;
    return;
  }

  if (reply_saved_to_flash) {
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::SETTINGS_SAVED_TO_FLASH,
    });
    reply_saved_to_flash = false;
    return;
  }
}

void receive_data(uint8_t * rx_buffer, size_t rx_size) {
  // We have a partial message that has timed out; reset the buffer and continue 
  // reading as a fresh message.
  if (encoding_buffer.decode_ongoing() and 
    (HAL_GetTick() - last_receive_time) > partial_message_timeout_ms) {
    reset_buffers();
  }

  encoding_buffer.decode_chunk(rx_buffer, rx_size, [&](uint8_t * buffer, size_t size){
    hex_mini_drive::Message message;
    if(hex_mini_drive::read_message(message, buffer, size)){
      // We have a valid message; handle it.
      handle_message(message);
    } else {
      // Invalid message; ignore it.
      bytes_discarded += size;
    }
  });
  last_receive_time = HAL_GetTick();
}

void comms_update(hex_mini_drive::FullReadout const& readout) {
  // Queue the state readouts on the USB buffer.
  queue_response(readout);

  const bool update_success = usb_update(
    encoding_buffer.encoding_buffer, encoding_buffer.encoding_buffer_size, 
    [&](uint8_t * buffer, size_t size){
      receive_data(buffer, size);
    }
  );

  if (update_success) {
    bytes_sent += encoding_buffer.encoding_buffer_size;

    // Data was queued; clear the response buffer.
    encoding_buffer.encode_reset();
    last_send_time = HAL_GetTick();
  } else if (encoding_buffer.is_message_encoded()) {
    // We have data to send but the queue is busy.
    if(HAL_GetTick() - last_send_time > sending_timeout_ms){
      // Timeout waiting for send; reset the buffers.
      reset_buffers();

      return usb_reset();
    }
  }
}

// Main app loop
// -------------

void app_tick() {
  main_loop_number += 1;

  hex_mini_drive::FullReadout readout = get_readout();

  // Limits
  // ------

  const uint16_t readouts_since_last_tick = (readout_number_base + readout.readout_number - last_readout_number) % readout_number_base;

  last_readout_number = readout.readout_number;
  
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

  set_live_max_pwm(
    clip_to(0, pwm_max, readout.live_max_pwm + pwm_penalty)
  );


  // Timing
  // ------

  // Update timing information.
  const uint32_t milliseconds = HAL_GetTick();

  const uint32_t duration_since_timing_update = milliseconds - last_update_time_millis;
  if (duration_since_timing_update > min_timing_period_millis) {
    last_update_time_millis = milliseconds;
    
    float seconds = duration_since_timing_update / 1000.f;

    main_loop_rate = (main_loop_number - timing_loop_number) / seconds;
    adc_update_rate = ((readout_number_base + readout.readout_number - timing_readout_number) % readout_number_base) / seconds;

    timing_loop_number = main_loop_number;
    timing_readout_number = readout.readout_number;
  }

  
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

  // Only compute the motor constant if we have a valid angle and the EMF voltage is above the threshold where noise is low.
  const bool compute_motor_constant = readout.angle_fix and (readout.emf_voltage_magnitude > control_parameters.min_emf_for_motor_constant);

  // Get the error (gradient) for the motor constant observer.
  const float motor_constant_error = compute_motor_constant * (readout.emf_voltage_magnitude - predicted_emf_voltage);

  const float motor_constant = readout.motor_constant + motor_constant_error * control_parameters.motor_constant_ki;

  
  // Calibration calculations
  // ------------------------

  if (
    calibration_mode != CalibrationMode::NONE and
    not readout_history_get_reset_flag() and
    (get_readout_history_size() >= hex_mini_drive::HISTORY_SIZE)
  ) {
    // We are in calibration mode; run the calibration calculations.
    // 
    // The calibration is done in the driver loop, but we need to run the calculations here
    // to update the calibration values and send them to the host.

    // Check if the calibration is complete.
    // 
    // The calibration starts after the reset flag is cleared, and ends when the motor returns to
    // a safe state.

    const float learning_rate = 0.0f;

    // Get data from history and compute gradients and the mean of the samples.
    // Then run gradient descent on the resistance and inductance values.
    hex_mini_drive::Readout const* readout_history = get_readout_history();
    const size_t history_size = get_readout_history_size();
    for (size_t i = 0; i < history_size; ++i) {
      hex_mini_drive::Readout const& readout = readout_history[i];
      
      // Tag unused variables.
      (void)readout; // Tag unused variable.
      (void)learning_rate; // Tag unused variable.
    }

    // Done an iteration of the calibration (it should take about 10 for a good value and 90 to stabilise).
    calibration_mode = CalibrationMode::NONE;
  }

  

  // Write all values
  // ----------------
  
  readout.main_loop_rate = main_loop_rate;
  readout.adc_update_rate = adc_update_rate;
  
  // TODO: make sure these are updated in the interrupts.
  // readout.resistance = current_calibration.resistance;
  // readout.inductance_inverse = current_calibration.inductance_inverse;
  // readout.inductance_bias = current_calibration.inductance_bias;
  // readout.inductance_bias_angle = current_calibration.inductance_bias_angle;
  // readout.saturation_angle = current_calibration.saturation_angle;
  // readout.saturation_factor = current_calibration.saturation_factor;
  readout.motor_constant = motor_constant;

  // Adjust direction
  // ----------------

  readout.angle = control_parameters.motor_direction * readout.angle;
  readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;
  readout.rotations = control_parameters.motor_direction * readout.rotations;

  // Comms update
  comms_update(readout);
}



// TODO: Reimplement the advanced motor control algorithms in the main loop.
// -------------------------------------------------------------------------

// // The complete driver state, these values control the motor behaviour.
// struct DriverState {
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


// // Drive the motor with the desired output current (note the DQ0 current is 3/2 phase current).
// // This mode also uses the smooth driving algorithm to drive using field oriented control, thus
// // keeping the current as close to 90 degrees ahead of the magnetic angle as possible. Allowing
// // control of the torque produced by the motor which will be proportional to the current target.
// static inline MotorOutputs update_motor_torque(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
    
//     // Squash very low currents to 0 to avoid noise.
//     const bool current_detected = readout.state_flags & current_detected_bit_mask;
    
//     // Get the signed current magnitude to compare against the target.
//     const float measured_current = current_detected * readout.current_magnitude * get_sin(readout.current_angle_offset);

//     // Calculate the difference between the target and measured current.
//     const float control_error = (driver_state.target - measured_current) * max_drive_current_inverse;

//     // Update the PID control for the torque.
//     driver_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         driver_state.active_pwm + control_error * control_parameters.torque_control_ki
//     );

//     return update_motor_smooth(driver_state, readout);
// }

// // Drive motor using up to a target battery power consumption.
// // 
// // The sign of the target power determines the direction of driving.
// // 
// // This mode will also control power absorption during breaking.
// static inline MotorOutputs update_motor_battery_power(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){

//     const float measured_power = sign(readout.angular_speed) * readout.total_power;

//     const float control_error = (driver_state.target - measured_power) * max_drive_power_inverse;

//     // Update the PID control for the torque.
//     driver_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         driver_state.active_pwm + control_error * control_parameters.battery_power_control_ki
//     );

//     return update_motor_smooth(driver_state, readout);
// }


// static inline MotorOutputs update_motor_speed(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // Calculate the difference between the target and measured current.
//     const float control_error = (driver_state.target - readout.angular_speed) * max_angular_speed_inverse;

//     // Update the PID control for the torque.
//     driver_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         driver_state.active_pwm + control_error * control_parameters.speed_control_ki
//     );

//     return update_motor_smooth(driver_state, readout);
// }

// static inline MotorOutputs update_motor_torque_speed(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     const float command_sign = sign(driver_state.speed_target);
    
//     // Squash very low currents to 0 to avoid noise.
//     const bool current_detected = readout.state_flags & current_detected_bit_mask;
    
//     // Get the signed current magnitude to compare against the target.
//     const float measured_current = current_detected * readout.current_magnitude * get_sin(readout.current_angle_offset);
    
//     // Calculate the difference between the target and measured current.
//     const float current_error = (driver_state.current_target - measured_current) * 
//         max_drive_current_inverse * control_parameters.torque_control_ki;

//     // Calculate the difference between the target and measured current.
//     const float speed_error = (driver_state.speed_target - readout.angular_speed) * 
//         max_angular_speed_inverse * control_parameters.speed_control_ki;


//     const float control_error = (
//         sign(current_error) != command_sign ? current_error :
//         sign(speed_error) != command_sign ? speed_error :
//         current_error + speed_error
//     );

//     // Update the PID control for the torque.
//     driver_state.target_pwm = clip_to(
//         -pwm_max,
//         +pwm_max,
//         driver_state.active_pwm + control_error
//     );

//     return update_motor_smooth(driver_state, readout);
// }


// static inline MotorOutputs update_motor_seek_angle(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // The derivative of the error is negative the angular speed.
//     const float position_error_derivative = -readout.angular_speed;
    
//     // Get the error between the target angle and the current angle; predicted to
//     // a future position determined by prediction parameter.
//     const float rotations_error = driver_state.seek_angle.target_rotation - readout.rotations;

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
//         driver_state.seek_integral + control_parameters.seek_ki * position_error
//     );

//     // Update the output of the Proportional Integral Derivative (PID) control.
//     float output = (proportional + integral + derivative);
    
//     // Only accumulate the integral if the output isn't saturated.

//     if (output > +1.0) {
//         // Decay the integral if we are saturating the output.
//         driver_state.seek_integral -= driver_state.seek_integral * seek_integral_decay_fraction;
//         // Maximum output.
//         output = +1.0;
//     } else if (output < -1.0) {
//         // Decay the integral if we are saturating the output negatively.
//         driver_state.seek_integral -= driver_state.seek_integral * seek_integral_decay_fraction;
//         // Minimum output.
//         output = -1.0;
//     } else {
//         // Output is not saturated, we can update the integral either direction.
//         driver_state.seek_integral = integral;
//     }

//     driver_state.current_target = output * driver_state.target;
//     driver_state.speed_target = sign(driver_state.current_target) * faster_abs(driver_state.speed_target);

//     return update_motor_torque_speed(driver_state, readout);
// }

// // Drive the motor using a fixed schedule for the PWM outputs.
// static inline MotorOutputs update_motor_schedule(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     PWMSchedule const& schedule = *driver_state.schedule.pointer;
//     PWMStage const& schedule_stage = schedule[driver_state.schedule.current_stage];

//     driver_state.schedule.stage_counter += 1;

//     if (driver_state.schedule.stage_counter >= schedule_stage.duration) {
//         // Move to the next stage in the schedule.
//         driver_state.schedule.current_stage += 1;
//         driver_state.schedule.stage_counter = 0;
//     }
    
//     return MotorOutputs{
//         .u_duty = static_cast<uint16_t>(schedule_stage.u_duty * driver_state.target),
//         .v_duty = static_cast<uint16_t>(schedule_stage.v_duty * driver_state.target),
//         .w_duty = static_cast<uint16_t>(schedule_stage.w_duty * driver_state.target)
//     };
// }



// // For the resistance calibration we will drive the motor U V and W phases with a pyramid waveform
// // and measure the current response to determine the resistance of the motor windings.
// static inline MotorOutputs update_motor_resistance_calibration(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // The total duration is HISTORY_SIZE, let's divide it into 6 segments where we
//     // drive the current at 0, 60, 120, 180, 240, and 300 degrees phase angle. For
//     // each phase divide the duration into 3 segments, first we stay at 0, then ramp
//     // up to the target PWM, then ramp down to 0.

//     // Number of PWM cycles elapsed since the calibration started.
//     const int32_t elapsed = hex_mini_drive::HISTORY_SIZE - driver_state.duration;

//     // Duration of a single phase angle segment.
//     const int32_t segment_duration = hex_mini_drive::HISTORY_SIZE / 6;

//     // Which of the 6 phase angle segments we are currently in.
//     const int32_t segment_index = elapsed / segment_duration;

//     // How far we are into the current segment.
//     const int32_t segment_progress = elapsed - segment_index * segment_duration;

//     // Duration of a single ramp (a third of the segment).
//     const int32_t ramp_duration = segment_duration / 3;

//     // Drive the phases at 0, 60, 120, 180, 240, and 300 degrees.
//     driver_state.active_angle = static_cast<int32_t>(segment_index * (angle_base / 6));

//     // Build a pyramid waveform: hold at 0, ramp up to the target PWM, then ramp back down to 0.
//     const float abs_pwm = (
//         segment_progress < ramp_duration ? 0 :
//         segment_progress < 2 * ramp_duration ? driver_state.target * (segment_progress - ramp_duration) / ramp_duration :
//         driver_state.target * (3.0 * ramp_duration - segment_progress) / ramp_duration
//     );

//     driver_state.active_pwm = clip_to(0.0f, driver_state.target, abs_pwm);

//     return update_motor_at_angle(driver_state, readout);
// }

// // Drive the motor using large step increases to measure the inductance of the motor windings.
// static inline MotorOutputs update_motor_inductance_calibration(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     // For the inductance calibration we will also drive the 3 motor phases, however this time
//     // we will do so in large steps and at opposite poles. This time split the HISTORY_SIZE total
//     // into 3 segments, each phase is further divided into 6 segments as such, assuming we start at 0: 
//     // first a step to half target PWM then a step to negative half target then step to 0,
//     // then step to positive target PWM then a step to negative target PWM then back to 0.
    
//     // Number of PWM cycles elapsed since the calibration started.
//     const int elapsed = hex_mini_drive::HISTORY_SIZE - driver_state.duration;

//     // Duration of a single phase angle segment (one per phase: 0, 120, and 240 degrees).
//     const int segment_duration = hex_mini_drive::HISTORY_SIZE / 3;

//     // Which of the 3 phase angle segments we are currently in.
//     const int segment_index = elapsed / segment_duration;

//     // How far we are into the current segment.
//     const int segment_progress = elapsed - segment_index * segment_duration;

//     // Duration of a single step (a sixth of the segment).
//     const int step_duration = segment_duration / 6;

//     // Which of the 6 steps we are currently in.
//     const int step_index = segment_progress / step_duration;

//     // Drive the phases at 0, 120, and 240 degrees.
//     driver_state.active_angle = static_cast<int32_t>(segment_index * (angle_base * 0.33333333));

//     // Half of the target PWM used for the first two steps.
//     const int half_pwm = driver_state.target / 2;

//     // Build the step waveform: +half, -half, 0, +full, -full, 0. Negative PWM drives the
//     // opposite pole via update_motor_at_angle flipping the angle by half a circle.
//     driver_state.active_pwm = (
//         step_index == 0 ? +half_pwm :
//         step_index == 1 ? -half_pwm :
//         step_index == 3 ? +driver_state.target :
//         step_index == 4 ? -driver_state.target :
//         0
//     );

//     return update_motor_at_angle(driver_state, readout);
// }

// // The chirp test drives the inductor at pwm_target around the coils at the requested speed
// // and number of rotations. Because the chirp is fast and the motor is presumed stationary,
// // we can measure the residual voltage terms after accounting for resistance and inductance
// // to isolate the response of the iron core of the coils. We can use this to determine the
// // rotor position.
// static inline MotorOutputs update_motor_rotating_calibration_chirp(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){
//     if (driver_state.test_parameters.test_duration > 0) {
//         driver_state.test_parameters.test_duration -= 1;

//         // Drive at the requested PWM until the test is done.
//         driver_state.active_angle += static_cast<int32_t>(driver_state.test_parameters.test_speed);
//         driver_state.active_pwm = faster_abs(driver_state.target);
//         return update_motor_at_angle(driver_state, readout);
//     } else {
//         return breaking_motor_outputs;
//     }
// }

// // In this test we chirp the motor in amplitude along the active_angle axis.
// static inline MotorOutputs update_motor_fixed_calibration_chirp(
//     DriverState & driver_state,
//     hex_mini_drive::FullReadout const& readout
// ){

//     if (driver_state.test_parameters.test_duration > 0) {
//         driver_state.test_parameters.test_duration -= 1;

//         // Reuse test_angle for the amplitude oscilation angle.
//         driver_state.test_parameters.test_angle += static_cast<int32_t>(driver_state.test_parameters.test_speed);

//         // Drive at a sinusoidal PWM amplitude along the active_angle axis until the test is done.
//         driver_state.active_pwm = faster_abs(driver_state.target) * get_sin(driver_state.test_parameters.test_angle);

//         return update_motor_at_angle(driver_state, readout);
//     } else {
//         // Record the shorted outputs.
//         return breaking_motor_outputs;
//     }
// }

// constexpr int16_t hfi_duration = 8;
// constexpr int16_t hfi_skips = (5 /* milliseconds */ * pwm_cycles_per_second / 1000) / hfi_duration;

// static inline MotorOutputs update_motor_hfi_saliency_pulses(
//     DriverState & driver_state,
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
//                 driver_state.active_pwm = -driver_state.target;
//                 driver_state.active_angle = angle_from_saliency + quarter_circle/2;
//                 break;
//             case 1:
//                 driver_state.active_pwm = +driver_state.target;
//                 driver_state.active_angle = angle_from_saliency + quarter_circle/2;
//                 break;
//             case 2:
//                 driver_state.active_pwm = +driver_state.target;
//                 driver_state.active_angle = angle_from_saliency - quarter_circle/2;
//                 break;
//             case 3:
//                 driver_state.active_pwm = -driver_state.target;
//                 driver_state.active_angle = angle_from_saliency - quarter_circle/2;
//                 break;
//         }
//     } else {
//         driver_state.active_pwm = 0;
//     }
//     return update_motor_at_angle(driver_state, readout);
// }

