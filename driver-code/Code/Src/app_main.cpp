#include "app_main.hpp"

#include "interrupts.hpp"
#include "io.hpp"
#include "usb_com.hpp"
#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "constants.hpp"
#include "dq0_transform.hpp"

#include <cstdint>
#include <cstring>
#include <memory>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_ll_cordic.h>


// Last known motor values
// -----------------------

int32_t angle = 0;


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



// Calibration state
// -----------------

// Current calibration result.
hex_mini_drive::CalibrationInstance calibration_instance = {};

struct CalibrationInstanceSettings {
  // Number of calibration steps remaining.
  size_t steps_remaining;
  // Target value for the series.
  float target;
  // Speed for the series.
  float speed;
};

// Track calibration settings.
CalibrationInstanceSettings calibration_instance_settings;

const uint16_t calibration_duration = max_duration / 2;

const float calibration_target_increment = 32.f;

const float calibration_target_max = 256.f;

const float calibration_speed_increment = 100.f * angle_base / static_cast<float>(pwm_cycles_per_second);

const float calibration_speed_max = angle_base / 2.f;


// Trying resilient backpropagation again, this time in the main loop.
// 
// We multiply the learning rates when we see consecutive updates with the same sign.
// And we decrease the mutliplier when the sign oscillates, this is called resilient
// backpropagation and it's relatively robust to the choice of initial learning rate
// but also really fast!
// 
// We can also declare that we have stable calibration parameters when the learning
// rate multiplier drops below a threshold.
  
const size_t batch_size = 32;

const float minimum_multiplier = 0.01f;
const float stabile_multiplier = 0.1f;
const float maximum_multiplier = 1000.f;

const float increase_factor = 1.2f;
const float decrease_factor = 0.7f;

hex_mini_drive::CalibrationInstance gradient_accumulator;
hex_mini_drive::CalibrationInstance gradient_count;
hex_mini_drive::CalibrationInstance update_sign;


static inline float resilient_gradient(
  const float gradient,
  float & gradient_accumulator,
  float & gradient_count,
  float & update_sign,
  float & learning_rate_multiplier
) {
  gradient_accumulator += gradient;
  gradient_count += 1;
  
  if (gradient_count < batch_size) return 0.f;

  const float gradient_sum = gradient_accumulator;
  gradient_accumulator = 0.f;
  gradient_count = 0.f;

  // No way it's exactly 0 right?!
  if (gradient_sum == 0.f) return 0.f;

  const float previous_update_sign = update_sign;
  update_sign = gradient_sum < 0 ? -1.f : 1.f;

  if (previous_update_sign == update_sign) {
    learning_rate_multiplier = min(learning_rate_multiplier * increase_factor, maximum_multiplier);
  } else if (previous_update_sign * update_sign < 0) {
    learning_rate_multiplier = max(learning_rate_multiplier * decrease_factor, minimum_multiplier);
  }

  return learning_rate_multiplier * gradient_sum;
}

static inline void start_calibration_series(
  hex_mini_drive::Calibration const& active_calibration,
  float target,
  float speed
) {
  
  // Reset the calibration instance state and initialize with the currently active calibration.

  calibration_instance = hex_mini_drive::CalibrationInstance {
    .count = 0,
    .target_pwm = target,
    .target_speed = speed,
    .resistance_x = active_calibration.resistance,
    .resistance_x_multiplier = 1.f,
    .resistance_y = 0,
    .resistance_y_multiplier = 1.f,
    .inductance_x = active_calibration.inductance,
    .inductance_x_multiplier = 1.f,
    .inductance_y = 0,
    .inductance_y_multiplier = 1.f,
    .inductance_saliency = active_calibration.inductance_bias,
    .inductance_saliency_multiplier = 1.f,
    .saliency_angle = 2 * angle,
  };
    
  gradient_accumulator = {
    .resistance_x = 0.f,
    .inductance_x = 0.f,
    .inductance_saliency = 0.f
  };

  gradient_count = {
    .resistance_x = 0.f,
    .inductance_x = 0.f,
    .inductance_saliency = 0.f
  };

  update_sign = {
    .resistance_x = 0.f,
    .inductance_x = 0.f,
    .inductance_saliency = 0.f
  };

  drive_state_queue.write(hex_mini_drive::DriveState{
    .motor_flags = ENABLE_ALL_OUTPUTS,
    .closed_loop = false,
    .duration = calibration_duration,
    .target_value = target,
    .angle_value = angle,
    .target_speed = speed,
    .probing_factor = 0,
    .probing_factor_modulation = 0,
    .probing_speed_modulation = 0,
    .probing_offset = 0,
  });
}


// Main loop control modes
// -----------------------

enum struct ControlMode : uint8_t {
  IDLE,
  DRIVE_SCHEDULE,
  SEEK_ANGLE,
  CALIBRATE
};

ControlMode control_mode = ControlMode::DRIVE_SCHEDULE;

// Switch to idle control mode and cancel any ongoing mode.
static inline void control_mode_idle(){
  drive_state_queue.mark_all_read();
  control_mode = ControlMode::IDLE;
}

static inline void control_mode_drive_state(hex_mini_drive::DriveState const& drive_state) {
  control_mode_idle();
  drive_state_queue.write(drive_state);
  control_mode = ControlMode::DRIVE_SCHEDULE;
}

static inline void control_mode_drive_schedule(hex_mini_drive::DriveSchedule const& drive_schedule) {
  control_mode_idle();
  size_t i = 0;
  for (; i < hex_mini_drive::DRIVE_SCHEDULE_SIZE; i++) {
    auto const& drive_state = drive_schedule[i];
    // The schedule lasts until the first state with 0 duration.
    if (drive_state.duration == 0) break;
    drive_state_queue.write_at_offset(i, drive_state);
  }
  drive_state_queue.mark_written(i);
 
  control_mode = ControlMode::DRIVE_SCHEDULE;
}

static inline void control_mode_calibrate(hex_mini_drive::Calibration const& active_calibration){
  control_mode_idle();
  start_calibration_series(active_calibration, calibration_target_increment, 0.f);
  control_mode = ControlMode::CALIBRATE;
}

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

    case SET_STATE_OFF: {
      control_mode_idle();
      return;
    }

    case SET_STATE_FREEWHEEL: {
      control_mode_drive_state(freewheel_drive_state);
      return;
    }

    case SET_DRIVE_STATE: {
      control_mode_drive_state(std::get<DriveState>(message.message_data));
      return;
    }

    case SET_DRIVE_SCHEDULE: {
      control_mode_drive_schedule(std::get<DriveSchedule>(message.message_data));
      return;
    }

    case SET_STATE_DRIVE_6_SECTOR: {
      // set_motor_command(DriveState{ 
      //   .mode = DriverMode::DRIVE_6_SECTOR, 
      //   .duration = std::get<BasicDriveCommand>(message.message_data).duration, 
      //   .active_pwm = std::get<BasicDriveCommand>(message.message_data).pwm_value * control_parameters.motor_direction,
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

    case RUN_CALIBRATION: {
      control_mode_calibrate(calibration);
      return;
    }

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
    case CALIBRATION_INSTANCE:
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


static inline void finish_calibration_series(){
  queue_message(hex_mini_drive::Message{
    .message_code = hex_mini_drive::MessageCode::CALIBRATION_INSTANCE,
    .message_data = calibration_instance
  });
  
  float target = calibration_instance.target_pwm;
  float speed = calibration_instance.target_speed + calibration_speed_increment;

  if (speed > calibration_speed_max) {
    // Increment target pwm if we reach the end of the speed scanning.
    speed = 0;
    target += calibration_target_increment;

    // Finish calibration mode and return to idle when we have scanned through all target PWM values.
    if (target > calibration_target_max) return control_mode_idle();
  }

  // We have another series to run if we've reach this spot, set it up and go!
  start_calibration_series(calibration, target, speed);
}

static inline void calibrate_from_readout(hex_mini_drive::Readout const& readout) {
  // Calibration checks
  // ------------------
  // 
  // For the remaining parameters we can only optimize them when there's no EMF voltage because
  // of the way we've set our equations. We detect the EMF voltage as basically the error term
  // of our electrical equations so if we optimize resistance and inductance while the motor spins
  // then we're going to slowly attribute all of the resistance and inductance voltage to the EMF.
  // 
  // It appears that it's really difficult to gauge when we can calibrate and when we cannot because
  // of EMF. Instead we will only calibrate when commanded and the driver is realatively certain the
  // motor is in a stopped state. We then run the calibration at various frequencies and check the
  // response assuming the motor is stationary and not generating any EMF voltage.

  
  // Stop calibration updates when we are undervoltage or overcurrent, we won't get accurate readings.
  const bool undervoltage = readout.vcc_voltage < control_parameters.vcc_undervoltage;
  const bool overcurrent = readout.current_magnitude > max_drive_current;

  // Check when we finished the current calibration series by seeing when the drive state has finished.
  const bool series_finished = not drive_state_queue.read_available();

  // Finish the current calibration series if trigger any stop condition or the probing just ends naturally.
  if (undervoltage or overcurrent or series_finished) return finish_calibration_series();

  // Because our best source of voltage information is the drive_voltage we also need to make sure
  // we are driving with enough power to get any information.

  const float drive_voltage_square = square(readout.direct_drive_voltage) + square(readout.quadrature_drive_voltage);

  const bool low_drive = drive_voltage_square < control_parameters.voltage_measurement_variance;

  // The rest of the calibration can only be done if we're actively driving the motor phases as that
  // is our only source of reference scale for the phase voltages. In any case, idle driving isn't an
  // error so we continue the calibration, we just ignore this readout.
  if (low_drive) return;


  // From now on we are definitely calibrating.
  calibration_instance.count += 1;


  // Recalculate
  // -----------
  // We need to recalculate these values in the main loop as they are not the primary tracking variables.
  

  // Calculate the resistive voltage drop across the coil and MOSFET resistance.
  const float direct_resistive_voltage = readout.direct_current * calibration_instance.resistance_x;
  const float quadrature_resistive_voltage = readout.quadrature_current * calibration_instance.resistance_x;

  // And the current diffs for calculating inductance.
  const float direct_diff = readout.direct_current_diff * current_diff_to_voltage_units;
  const float quadrature_diff = readout.quadrature_current_diff * current_diff_to_voltage_units;

  // Note: The saliency effect is due to the iron between magnets so it occurs twice per 
  // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.

  const float cos_saliency = get_cos(calibration_instance.saliency_angle);
  const float sin_saliency = get_sin(calibration_instance.saliency_angle);

  const float direct_inductance_bias_voltage = calibration_instance.inductance_saliency * (
      -direct_diff * cos_saliency 
      -quadrature_diff * sin_saliency
  );
  const float quadrature_inductance_bias_voltage = calibration_instance.inductance_saliency * (
      -direct_diff * sin_saliency
      +quadrature_diff * cos_saliency
  );

  const float direct_inductance_base_voltage = direct_diff * calibration_instance.inductance_x;
  const float quadrature_inductance_base_voltage = quadrature_diff * calibration_instance.inductance_x;

  const float direct_inductor_voltage = direct_inductance_base_voltage + direct_inductance_bias_voltage;
  const float quadrature_inductor_voltage = quadrature_inductance_base_voltage + quadrature_inductance_bias_voltage;
  
  // Calibrate without EMF!!!
  // ------------------------
  // 
  // It's very important that we don't calibrate while the motor is rotating and generating EMF.
  // During sensorless running we infer the EMF as the error of the electrical equation, but
  // that means we can't use the inferred EMF to calibrate the electrical parameters accurately.
  // Especially because the resistance voltage and EMF voltage are covarying quantities.
  // Inductance and the saliency bias are also highly sensitive to any lag in the EMF estimation.

  const float direct_voltage_error = direct_inductor_voltage + direct_resistive_voltage - readout.direct_drive_voltage;
  const float quadrature_voltage_error = quadrature_inductor_voltage + quadrature_resistive_voltage - readout.quadrature_drive_voltage;

  // Defining the loss as the square(direct_voltage_error) + square(quadrature_voltage_error) we get the derivative gradients with 
  // respect to each of the parameters we're tracking. We only care about the sign so we should be careful to skip updates when 
  // there isn't enough energy in the error to warrant an update.
  // const float square_voltage_error = square(direct_voltage_error) + square(quadrature_voltage_error);

  // Gradients & Parameter Updates
  // -----------------------------
  // 
  // Compute the gradients of the loss with respect to each of the tracked parameters.
  // 
  // ! We must not condition the updates apparently...

  const float resistance_gradient = (
      direct_voltage_error * direct_resistive_voltage +
      quadrature_voltage_error * quadrature_resistive_voltage
  ) * volts_per_voltage_units_square;

  const float resilient_resistance_gradient = resilient_gradient(
      resistance_gradient,
      gradient_accumulator.resistance_x,
      gradient_count.resistance_x,
      update_sign.resistance_x,
      calibration_instance.resistance_x_multiplier
  );
  
  calibration_instance.resistance_x = validate_parameter(
    calibration_instance.resistance_x - resilient_resistance_gradient * control_parameters.resistance_ki,
    minimum_resistance
  );
  
  const float inductance_gradient = (
    direct_voltage_error * direct_inductance_base_voltage +
    quadrature_voltage_error * quadrature_inductance_base_voltage
  ) * volts_per_voltage_units_square;

  const float resilient_inductance_gradient = resilient_gradient(
    inductance_gradient,
    gradient_accumulator.inductance_x,
    gradient_count.inductance_x,
    update_sign.inductance_x,
    calibration_instance.inductance_x_multiplier
  );
  
  // Keep 0 inductance exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
  calibration_instance.inductance_x = validate_parameter(
    calibration_instance.inductance_x - resilient_inductance_gradient * control_parameters.inductance_ki,
    minimum_inductance
  );

  const float inductance_bias_gradient = (
    direct_voltage_error * direct_inductance_bias_voltage +
    quadrature_voltage_error * quadrature_inductance_bias_voltage
  ) * volts_per_voltage_units_square;

  const float resilient_inductance_bias_gradient = resilient_gradient(
    inductance_bias_gradient,
    gradient_accumulator.inductance_saliency,
    gradient_count.inductance_saliency,
    update_sign.inductance_saliency,
    calibration_instance.inductance_saliency_multiplier
  );
  
  // Keep 0 inductance bias exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
  calibration_instance.inductance_saliency = validate_parameter(
    calibration_instance.inductance_saliency - resilient_inductance_bias_gradient * control_parameters.inductance_bias_ki,
    minimum_inductance_bias
  );

  const float saliency_angle_gradient = (
    direct_voltage_error * -quadrature_inductance_bias_voltage +
    quadrature_voltage_error * direct_inductance_bias_voltage
  ) * volts_per_voltage_units_square;

  const float saliency_angle_change = saliency_angle_gradient * control_parameters.saliency_angle_ki;
  calibration_instance.saliency_angle -= static_cast<int32_t>(saliency_angle_change);
}

static inline void process_readout(hex_mini_drive::Readout const& readout) {
  // Count if we skip any readouts.
  // const uint16_t readouts_since_last_tick = (readout_number_base + readout.readout_number - last_readout_number) % readout_number_base;

  last_readout_number = readout.readout_number;

  angle = readout.angle;

  // TODO: redo the saliency angle adjustments (we might wanna do this in the drive loop i reckon, and leave the rest of the calibration logic here).
  // // Accumulate angle adjustments and send over until we reduce the saliency_angle_offset to 0.
  // const int32_t pending_adjustment = (info.saliency_angle_offset / 2) * control_parameters.angle_from_saliency_ki;

  // if (set_angle_adjustment(pending_adjustment)) {
  //   info.saliency_angle_offset -= 2 * pending_adjustment;
  // }

  // Update control
  // --------------
  
  // Keep track of the control mode and send drive_state commands to the interrupt loop if required.
  switch(control_mode) {
    // Do nothing while in IDLE mode.
    case ControlMode::IDLE: break;
    // Switch back to idle when the drive schedule has completed.
    case ControlMode::DRIVE_SCHEDULE: {
      if (not drive_state_queue.read_available()) control_mode = ControlMode::IDLE;
      break;
    }
    // Seek a particular position.
    case ControlMode::SEEK_ANGLE: {
      // TODO: implement
      break;
    }
    // Run calibration routine.
    case ControlMode::CALIBRATE: {
      // ! Only calibrate if we're in calibration mode.
      calibrate_from_readout(readout);
      break;
    }
  }

  // Current offsets
  // ---------------

  // Track the zero current offsets in the main loop.

  const bool emf_detected = square(readout.emf_voltage_magnitude) > readout.emf_magnitude_variance;

  // Only track the zero offset while there is no noticeable current being measured.
  if (not emf_detected and readout.current_magnitude < 2 * control_parameters.current_offset_maximum) {
    const auto currents = abc_transform(
      {readout.direct_current, readout.quadrature_current}, 
      readout.zero_current
    );

    // Finally if there's nothing unusual going on, we can update the zero offset for the currents.
    calibration.u_current_zero = clip_to(
      -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
      calibration.u_current_zero + std::get<0>(currents) * control_parameters.zero_current_ki
    );
    calibration.v_current_zero = clip_to(
      -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
      calibration.v_current_zero + std::get<1>(currents) * control_parameters.zero_current_ki
    );
    calibration.w_current_zero = clip_to(
      -control_parameters.current_offset_maximum, control_parameters.current_offset_maximum, 
      calibration.w_current_zero + std::get<2>(currents) * control_parameters.zero_current_ki
    );
  }

}


static inline void runtime_info_update(uint32_t milliseconds_time){
  // Run time stats
  // --------------

  // Count loop iterations.
  main_loop_number += 1;

  const uint32_t duration_since_timing_update = milliseconds_time - last_update_time_millis;
  if (duration_since_timing_update > min_timing_period_millis) {
    last_update_time_millis = milliseconds_time;
    
    float seconds = duration_since_timing_update / 1000.f;

    info.main_loop_rate = (main_loop_number - timing_loop_number) / seconds;
    info.adc_update_rate = ((readout_number_base + last_readout_number - timing_readout_number) % readout_number_base) / seconds;
    info.temperature = get_temperature();

    timing_loop_number = main_loop_number;
    timing_readout_number = last_readout_number;
  }
}

// Main app loop
// -------------

void app_tick() {
  // Get the current time in milliseconds.
  const uint32_t milliseconds_time = HAL_GetTick();
  
  // Update timing information.
  runtime_info_update(milliseconds_time);

  // Slow sensor updates
  // -------------------

  // Read new data from the hall sensors.
  // const uint8_t hall_state = read_hall_sensors_state();
    

  // Comms update
  // ------------

  usb_update();

  // It's been too long since we have received anything, discard any partial message.
  if (milliseconds_time - last_receive_time > partial_message_timeout_ms) {
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
    last_receive_time = milliseconds_time;
  }


  // Readout post-processing and streaming
  // -------------------------------------

  const size_t readouts_available = readout_history_queue.read_available();

  info.average_buffer_fill_rate += (readouts_available - info.average_buffer_fill_rate) * 0.01f;

  size_t i = 0;
  for (; i < readouts_available; i++) {
    // TODO: Adjust readout?
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

      if (control_mode == ControlMode::CALIBRATE) queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION_INSTANCE,
        .message_data = calibration_instance
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