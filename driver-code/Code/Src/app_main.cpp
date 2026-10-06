#include "app_main.hpp"

#include "interrupts.hpp"
#include "io.hpp"
#include "usb_com.hpp"
#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "constants.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_ll_cordic.h>


// Store the latest readout.
hex_mini_drive::FullReadout latest_readout;

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
bool reply_current_calibration = false;
bool reply_control_parameters = false;
bool reply_unit_test = false;
bool reply_saved_to_flash = false;

// Function signature for unit tests; the unit test prints text to the buffer.
using UnitTestFunction = void (*)(char * buffer, size_t max_size);

UnitTestFunction unit_test_function = nullptr;

hex_mini_drive::DriveSchedule pending_drive_schedule = {};

size_t pending_schedule_length = 0;

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

void break_motor(){
  pending_schedule_length = 0;
  while(not drive_state_queue.write_if_available(breaking_drive_state)){
    // Wait until we successfully write the breaking state to the queue.
  };
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

static inline hex_mini_drive::DriveState get_checked_drive_state(hex_mini_drive::DriveState const& drive_state) {
  return hex_mini_drive::DriveState{
    .motor_flags = drive_state.motor_flags,
    .enable_closed_loop = drive_state.enable_closed_loop,
    .duration = drive_state.duration,
    .pwm_value = clip_to(-pwm_max, +pwm_max, 
      drive_state.pwm_value * control_parameters.motor_direction),
    .angle = drive_state.angle,
    .angular_speed = clip_to(-max_angular_speed, max_angular_speed,
      drive_state.angular_speed * control_parameters.motor_direction
    ),
  };
}


// Handle the command on the buffer; returns whether there was an error.
void handle_message(hex_mini_drive::Message const& message) {

  using namespace hex_mini_drive;

  switch (message.message_code) {
    case NULL_MESSAGE_CODE:
      // No command received; ignore it.
      return;

    case STREAM_FULL_READOUTS: {
      // Continuously stream data if duration > 0.
      stream_state = std::get<StreamFullReadouts>(message.message_data).stream_state;
      // Also stop the motor if we stop the stream.
      if (not stream_state) break_motor();
      return;
    }

    // Turn off the motor driver.
    case SET_STATE_OFF: {
      // Repeat command until the driver stops.
      break_motor();
      return;
    }

    // Freewheel the motor.
    case SET_STATE_FREEWHEEL: {
      pending_schedule_length = 1;
      pending_drive_schedule[0] = freewheel_drive_state;
      return;
    }

    case SET_DRIVE_STATE: {
      pending_schedule_length = 1;
      auto const& drive_state = std::get<DriveState>(message.message_data);
      pending_drive_schedule[0] = get_checked_drive_state(drive_state);
      return;
    }

    case SET_DRIVE_SCHEDULE: {
      size_t i = 0;
      for (; i < hex_mini_drive::DRIVE_SCHEDULE_SIZE; i++) {
        auto const& drive_state = std::get<DriveSchedule>(message.message_data)[i];
        // The schedule lasts until the first state with 0 duration.
        if (drive_state.duration == 0) break;
        pending_drive_schedule[i] = get_checked_drive_state(drive_state);
      }
      pending_schedule_length = i;
      
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

    case GET_CURRENT_CALIBRATION:
      reply_current_calibration = true;
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

    case GET_CONTROL_PARAMETERS:
      reply_control_parameters = true;
      return;

    case SET_CONTROL_PARAMETERS:
      control_parameters = std::get<ControlParameters>(message.message_data);
      reply_control_parameters = true;
      return;

    case RESET_CONTROL_PARAMETERS:
      control_parameters = default_control_parameters;
      reply_control_parameters = true;
      return;

    case SET_ANGLE: {
      set_angle_offset(std::get<SetAngle>(message.message_data).angle - latest_readout.angle);
      return;
    }

    case SET_ROTATIONS: {
      set_rotations_offset(std::get<SetRotations>(message.message_data).rotations - latest_readout.rotations);
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
  }
}

void reset_buffers(){
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
  if(readout_history_queue.read_available() > 0){
    serialize_message(hex_mini_drive::Message{
      .message_code = hex_mini_drive::MessageCode::READOUT,
      .message_data = readout_history_queue.peek()
    });

    readout_history_queue.mark_read();

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

void comms_update() {
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

  full_readout_queue.read_if_available(latest_readout);

  auto & readout = latest_readout;

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

  // if (
  //   calibration_mode != CalibrationMode::NONE and
  //   not readout_history_get_reset_flag() and
  //   (get_readout_history_size() >= hex_mini_drive::HISTORY_SIZE)
  // ) {
  //   // We are in calibration mode; run the calibration calculations.
  //   // 
  //   // The calibration is done in the driver loop, but we need to run the calculations here
  //   // to update the calibration values and send them to the host.

  //   // Check if the calibration is complete.
  //   // 
  //   // The calibration starts after the reset flag is cleared, and ends when the motor returns to
  //   // a safe state.

  //   const float learning_rate = 0.0f;

  //   // Get data from history and compute gradients and the mean of the samples.
  //   // Then run gradient descent on the resistance and inductance values.
  //   hex_mini_drive::Readout const* readout_history = get_readout_history();
  //   const size_t history_size = get_readout_history_size();
  //   for (size_t i = 0; i < history_size; ++i) {
  //     hex_mini_drive::Readout const& readout = readout_history[i];
      
  //     // Tag unused variables.
  //     (void)readout; // Tag unused variable.
  //     (void)learning_rate; // Tag unused variable.
  //   }

  //   // Done an iteration of the calibration (it should take about 10 for a good value and 90 to stabilise).
  //   calibration_mode = CalibrationMode::NONE;
  // }

  

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
  // ------------

  // Queue the state readouts on the USB buffer.
  queue_response(readout);
  // Send and receive over the USB buffer.
  comms_update();

  // Handle driver state using last message
  // --------------------------------------
  
  // Write the whole message if we can.
  if (drive_state_queue.write_from_if_available(pending_drive_schedule.data(), pending_schedule_length)) {
    // We wrote it all, now reset the pending length.
    pending_schedule_length = 0;
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

