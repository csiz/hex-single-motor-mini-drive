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


// Main loop stats tracking
// ------------------------
uint32_t main_loop_number = 0;
uint32_t last_loop_number = 0;
uint16_t last_readout_number = 0;


const uint32_t min_timing_period_millis = 50;
uint32_t last_update_time_millis = 0;

float main_loop_rate = 0.0f;
float adc_update_rate = 0.0f;


float resistive_power_observer = 0.0f;
float total_power_observer = 0.0f;

// Calibration tracking
// --------------------

enum struct CalibrationMode {
  NONE,
  RESISTANCE,
  INDUCTANCE,
  POSITION_CHIRP,
  POSITION_EMF
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
bool reply_current_factors = false;
bool reply_control_parameters = false;
bool reply_unit_test = false;

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
  set_motor_command(DriverState{ 
    .mode = DriverMode::SCHEDULE,
    .target = value,
    .schedule = DriveSchedule{ .pointer = &schedule },
  });
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

      // Ping the driver loop to reset the history.
      set_motor_command(DriverState{.mode = DriverMode::CONTINUE});


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
    case SET_STATE_TEST_U_DIRECTIONS:
      motor_start_test(
        test_u_directions,
        std::get<TestCommand>(message.message_data).pwm_value,
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;

    case SET_STATE_TEST_U_INCREASING:
      motor_start_test(
        test_u_increasing, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_U_DECREASING:
      motor_start_test(
        test_u_decreasing, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_V_INCREASING:
      motor_start_test(
        test_v_increasing, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_V_DECREASING:
      motor_start_test(
        test_v_decreasing, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_W_INCREASING:
      motor_start_test(
        test_w_increasing, 
        std::get<TestCommand>(message.message_data).pwm_value, 
        std::get<TestCommand>(message.message_data).take_snapshot > 0
      );
      return;
    case SET_STATE_TEST_W_DECREASING:
      motor_start_test(
        test_w_decreasing, 
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
        .active_pwm = std::get<SetStateDrivePeriodic>(message.message_data).pwm_value,
        .target = std::get<SetStateDrivePeriodic>(message.message_data).angular_speed * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_SMOOTH: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_SMOOTH, 
        .duration = std::get<SetStateDriveSmooth>(message.message_data).timeout,
        .active_pwm = std::get<SetStateDriveSmooth>(message.message_data).pwm_value * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_TORQUE: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_TORQUE, 
        .duration = std::get<SetStateDriveTorque>(message.message_data).timeout,
        .target = std::get<SetStateDriveTorque>(message.message_data).target_current * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_BATTERY_POWER: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_BATTERY_POWER, 
        .duration = std::get<SetStateDriveBatteryPower>(message.message_data).timeout,
        .target = std::get<SetStateDriveBatteryPower>(message.message_data).target_power * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_DRIVE_SPEED: {
      set_motor_command(DriverState{
        .mode = DriverMode::DRIVE_SPEED, 
        .duration = std::get<SetStateDriveSpeed>(message.message_data).timeout,
        .target = std::get<SetStateDriveSpeed>(message.message_data).target_speed * control_parameters.motor_direction,
      });
      return;
    }

    case SET_STATE_SEEK_ANGLE_WITH_POWER: {
      set_motor_command(DriverState{
        .mode = DriverMode::SEEK_ANGLE_POWER, 
        .duration = std::get<SetStateSeekAngleWithPower>(message.message_data).timeout,
        .seek_angle = SeekAngle{
          .target_angle = std::get<SetStateSeekAngleWithPower>(message.message_data).target_angle * control_parameters.motor_direction,
          .target_rotation = std::get<SetStateSeekAngleWithPower>(message.message_data).target_rotation * control_parameters.motor_direction,
          .max_target = std::get<SetStateSeekAngleWithPower>(message.message_data).max_drive_power,
        }
      });
      return;
    }

    case SET_STATE_SEEK_ANGLE_WITH_TORQUE: {
      set_motor_command(DriverState{
        .mode = DriverMode::SEEK_ANGLE_TORQUE, 
        .duration = std::get<SetStateSeekAngleWithTorque>(message.message_data).timeout,
        .seek_angle = SeekAngle{
          .target_angle = std::get<SetStateSeekAngleWithTorque>(message.message_data).target_angle * control_parameters.motor_direction,
          .target_rotation = std::get<SetStateSeekAngleWithTorque>(message.message_data).target_rotation * control_parameters.motor_direction,
          .max_target = std::get<SetStateSeekAngleWithTorque>(message.message_data).max_drive_current,
        }
      });
      return;
    }

    case SET_STATE_SEEK_ANGLE_WITH_SPEED: {
      set_motor_command(DriverState{
        .mode = DriverMode::SEEK_ANGLE_SPEED, 
        .duration = std::get<SetStateSeekAngleWithSpeed>(message.message_data).timeout,
        .seek_angle = SeekAngle{
          .target_angle = std::get<SetStateSeekAngleWithSpeed>(message.message_data).target_angle * control_parameters.motor_direction,
          .target_rotation = std::get<SetStateSeekAngleWithSpeed>(message.message_data).target_rotation * control_parameters.motor_direction,
          .max_target = std::get<SetStateSeekAngleWithSpeed>(message.message_data).max_drive_speed,
        }
      });
      return;
    }

    // Freewheel the motor.
    case SET_STATE_FREEWHEEL:
      set_motor_command(DriverState{ .mode = DriverMode::FREEWHEEL });
      return;

    case SET_STATE_HOLD_U_POSITIVE: {
      set_motor_command(DriverState{ 
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_STATE_HOLD_V_POSITIVE: {
      set_motor_command(DriverState{ 
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_STATE_HOLD_W_POSITIVE: {
      set_motor_command(DriverState{ 
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_STATE_HOLD_U_NEGATIVE: {
      set_motor_command(DriverState{ 
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
          .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_STATE_HOLD_V_NEGATIVE: {
      set_motor_command(DriverState{ 
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
          .w_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_STATE_HOLD_W_NEGATIVE: {
      set_motor_command(DriverState{
        .motor_outputs = MotorOutputs{ 
          .enable_flags = enable_flags_all, 
          .u_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value)),
          .v_duty = static_cast<uint16_t>(faster_abs(std::get<HoldCommand>(message.message_data).pwm_value))
        },
        .mode = DriverMode::HOLD,
        .duration = std::get<HoldCommand>(message.message_data).timeout, 
      });
      return;
    }
    case SET_CURRENT_CALIBRATION:
      current_calibration = std::get<CurrentCalibration>(message.message_data);
      reply_current_factors = true;
      return;
        
    case RESET_CURRENT_CALIBRATION:
      // Reset the current factors to the default values.
      current_calibration = default_current_calibration;
      reply_current_factors = true;
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
      reply_current_factors = true;
      return;

    case GET_CONTROL_PARAMETERS:
      reply_control_parameters = true;
      return;

    // TODO: rename to SET_ANGLE_OFFSET
    case SET_ANGLE: {
      set_angle_offset(std::get<SetAngle>(message.message_data).angle);
      return;
    }

    case SAVE_SETTINGS_TO_FLASH:
      if(is_motor_safed()){
        save_settings_to_flash(current_calibration, control_parameters);

        current_calibration = get_current_calibration();
        control_parameters = get_control_parameters();
        
        return;
      }

    // We shouldn't receive these messages; the driver only sends them.
    case CURRENT_CALIBRATION:
    case CONTROL_PARAMETERS:
    case READOUT:
    case FULL_READOUT:
    case UNIT_TEST_OUTPUT:
      return;

    
    case SET_STATE_RESISTANCE_CALIBRATION: {
      calibration_mode = CalibrationMode::RESISTANCE;
      
      // Clear the readouts buffer of old data.
      readout_history_mark_reset();
      readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      readouts_sent = 0;

      set_motor_command(DriverState{
        .mode = DriverMode::RESISTANCE_CALIBRATION,
        .duration = hex_mini_drive::HISTORY_SIZE,
        .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      });
      return;
    }

    case SET_STATE_INDUCTANCE_CALIBRATION: {
      calibration_mode = CalibrationMode::INDUCTANCE;

      // Clear the readouts buffer of old data.
      readout_history_mark_reset();
      readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      readouts_sent = 0;
      
      set_motor_command(DriverState{
        .mode = DriverMode::INDUCTANCE_CALIBRATION,
        .duration = hex_mini_drive::HISTORY_SIZE,
        .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
      });
      return;
    }

    case SET_STATE_POSITION_CALIBRATION_CHIRP: {
      calibration_mode = CalibrationMode::POSITION_CHIRP;
      // Clear the readouts buffer of old data.
      readout_history_mark_reset();
      readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      readouts_sent = 0;

      set_motor_command(DriverState{
        .mode = DriverMode::POSITION_CALIBRATION_CHIRP,
        .duration = hex_mini_drive::HISTORY_SIZE,
        .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
        .test_parameters = TestParameters{
          .test_speed = std::get<TestCommand>(message.message_data).test_speed,
          .test_duration = std::get<TestCommand>(message.message_data).test_duration,
        },
      });
      return;
    }
    case SET_STATE_POSITION_CALIBRATION_EMF: {
      calibration_mode = CalibrationMode::POSITION_EMF;
      
      readout_history_mark_reset();
      readouts_to_send = (std::get<TestCommand>(message.message_data).take_snapshot > 0) ? hex_mini_drive::HISTORY_SIZE : 0;
      readouts_sent = 0;


      set_motor_command(DriverState{
        .mode = DriverMode::POSITION_CALIBRATION_EMF,
        .duration = hex_mini_drive::HISTORY_SIZE + std::get<TestCommand>(message.message_data).test_duration,
        .target = clip_to(0.0f, pwm_max, std::get<TestCommand>(message.message_data).pwm_value),
        .test_parameters = TestParameters{
          .test_speed = std::get<TestCommand>(message.message_data).test_speed,
          .test_duration = std::get<TestCommand>(message.message_data).test_duration,
        },
      });
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
  if (reply_current_factors) {
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::CURRENT_CALIBRATION,
      .message_data = current_calibration
    });
    reply_current_factors = false;
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
  // Update the higher resolution observer.
  resistive_power_observer += (readout.resistive_power - resistive_power_observer) * control_parameters.resistive_power_ki;

  // Calculate slowly varying averages of the total power; this represents the energy
  // drawn from the battery. At constant voltage, this is proportional to the current drawn.
  total_power_observer += (readout.total_power - total_power_observer) * control_parameters.power_draw_ki;


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
  const int pwm_penalty = max(
    vcc_mosfet_driver_undervoltage - readout.vcc_voltage,
    resistive_power_observer - control_parameters.max_resistive_power,
    total_power_observer - control_parameters.max_power_draw,
    faster_abs(readout.angular_speed) - control_parameters.max_angular_speed
  );

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


    ThreePhase resistance_gradient_step_sum = {0.0f, 0.0f, 0.0f};
    ThreePhase inductance_gradient_step_sum = {0.0f, 0.0f, 0.0f};

    const float learning_rate = 0.0f;

    // Get data from history and compute gradients and the mean of the samples.
    // Then run gradient descent on the resistance and inductance values.
    hex_mini_drive::Readout const* readout_history = get_readout_history();
    const size_t history_size = get_readout_history_size();
    for (size_t i = 0; i < history_size; ++i) {
      hex_mini_drive::Readout const& readout = readout_history[i];

      const ThreePhase drive_voltages = {readout.u_drive_voltage, readout.v_drive_voltage, readout.w_drive_voltage};
      
      const ThreePhase currents_prescaled = ThreePhase{
        readout.u_current,
        readout.v_current,
        readout.w_current
      } * current_to_voltage_units;

      const ThreePhase current_diffs_prescaled = ThreePhase{
        readout.u_current_diff,
        readout.v_current_diff,
        readout.w_current_diff
      } * current_diff_to_voltage_units;

      // Calculate the voltage drop across the coil inductance.
      const ThreePhase inductor_voltages = current_diffs_prescaled * current_calibration.inductance;

      // Calculate the resistive voltage drop across the coil and MOSFET resistance.
      const ThreePhase resistive_voltages = currents_prescaled * get_phase_resistances(current_calibration);

      // In the running loop we allocate all residual voltages to the EMF response, but in the calibration
      // modes we expect the motor to be nearly stationary so we can neglect the EMF and instead the diff
      // is the error residual due to our miscalibrated resistance and inductance values.
      const ThreePhase residual_voltages = inductor_voltages + resistive_voltages - drive_voltages;

      // Now that we have recalculated the values, we can calculate the gradients.

      const ThreePhase resistance_gradients = /* 2 * */residual_voltages * currents_prescaled;
      const ThreePhase inductance_gradients = /* 2 * */residual_voltages * current_diffs_prescaled;

      const ThreePhase resistance_2nd_gradients = /* 2 * */currents_prescaled * currents_prescaled;
      const ThreePhase inductance_2nd_gradients = /* 2 * */current_diffs_prescaled * current_diffs_prescaled;

      static const float current_measurement_variance_prescaled = current_measurement_variance * current_to_voltage_units;
      static const float current_diff_measurement_variance_prescaled = current_diff_measurement_variance * current_diff_to_voltage_units;

      const ThreePhase resistance_gradient_step = resistance_gradients / (resistance_2nd_gradients + three_same(current_measurement_variance_prescaled));
      const ThreePhase inductance_gradient_step = inductance_gradients / (inductance_2nd_gradients + three_same(current_diff_measurement_variance_prescaled));

      resistance_gradient_step_sum = resistance_gradient_step_sum + resistance_gradient_step;
      inductance_gradient_step_sum = inductance_gradient_step_sum + inductance_gradient_step;
    }

    static const float inverse_history_size = 1.0f / static_cast<float>(history_size);

    const ThreePhase resistance_gradient_step_mean = resistance_gradient_step_sum * inverse_history_size;
    const ThreePhase inductance_gradient_step_mean = inductance_gradient_step_sum * inverse_history_size;

    // Update the calibration values using gradient descent.
    static float dummy = 0.0f;

    dummy += learning_rate + std::get<0>(resistance_gradient_step_mean) + std::get<0>(inductance_gradient_step_mean);

    if (calibration_mode == CalibrationMode::RESISTANCE) {
      // current_calibration.u_resistance -= learning_rate * std::get<0>(resistance_gradient_step_mean);
      // current_calibration.v_resistance -= learning_rate * std::get<1>(resistance_gradient_step_mean);
      // current_calibration.w_resistance -= learning_rate * std::get<2>(resistance_gradient_step_mean);
    } else if (calibration_mode == CalibrationMode::INDUCTANCE) {
      // current_calibration.inductance -= learning_rate * (
      //   std::get<0>(inductance_gradient_step_mean) +
      //   std::get<1>(inductance_gradient_step_mean) +
      //   std::get<2>(inductance_gradient_step_mean)
      // ) * 0.333333f; // Average the inductance gradients across the three phases.
    } else if (calibration_mode == CalibrationMode::POSITION_CHIRP) {
      // TODO: implement position chirp calibration.
    } else if (calibration_mode == CalibrationMode::POSITION_EMF) {
      // TODO: implement position EMF calibration.
    }

    // Done an iteration of the calibration (it should take about 10 for a good value and 90 to stabilise).
    calibration_mode = CalibrationMode::NONE;
  }

  

  // Write all values
  // ----------------
  
  readout.main_loop_rate = main_loop_rate;
  readout.adc_update_rate = adc_update_rate;
  readout.motor_constant = motor_constant;

  // Adjust direction
  // ----------------

  readout.angle = control_parameters.motor_direction * readout.angle;
  readout.angle_adjustment = control_parameters.motor_direction * readout.angle_adjustment;
  readout.angular_speed = control_parameters.motor_direction * readout.angular_speed;

  readout.rotations = control_parameters.motor_direction * readout.rotations;
  readout.rotor_acceleration = control_parameters.motor_direction * readout.rotor_acceleration;

  // TODO: this doesn't get reported to the driver loop... we need to rethink that in a bit.
  readout.live_max_pwm = live_max_pwm;
  

  // Comms update
  comms_update(readout);
}