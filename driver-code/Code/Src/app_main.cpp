#include "app_main.hpp"

#include "interrupts.hpp"
#include "io.hpp"
#include "usb_com.hpp"
#include "parameters_store.hpp"
#include "math_utils.hpp"
#include "constants.hpp"
#include "sine_tables.hpp"
#include "dq0_transform.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_ll_cordic.h>


// Last known motor values
// -----------------------

int32_t angle = 0;
int32_t active_angle = 0;

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

const float calibration_target_increment = 128.f;

const float calibration_target_max = calibration_target_increment * 3;

// TODO: let's make sure high frequencies work nice before going lower.
const float calibration_speed_base = 4 * 110.f * angle_base / static_cast<float>(pwm_cycles_per_second); // Note A2 at 110 Hz.

const float calibration_speed_multiplier = std::pow(2, 1.f / 12.f); // An octave has 12 notes.

const float calibration_speed_max = angle_base / 10.f;

const float calibration_stats_ki = std::pow(2, -8);

const uint32_t calibration_stability_check_period = 128;

const float excitation_fraction_threshold = 0.2f;


static inline hex_mini_drive::CalibrationInstance init_calibration_instance(
  hex_mini_drive::CalibrationInstance const& active_calibration,
  float target,
  float speed
) {
  return hex_mini_drive::CalibrationInstance {
    .count = 0,
    .target_pwm = target,
    .target_speed = speed,
    .learning_rate_multiplier = 1.f,
    .learning_rate_resilience_sign = +1.f,
    .resistance_excitation_fraction = 0.f,
    .inductance_excitation_fraction = 0.f,
    .resistance_x = active_calibration.resistance_x,
    .resistance_y = active_calibration.resistance_y,
    .resistance_sensitivity = current_variance,
    .inductance = active_calibration.inductance,
    .inductance_sensitivity = inductance_variance,
    .inductance_saliency = active_calibration.inductance_saliency,
    .saliency_angle = active_calibration.saliency_angle,
    .hysteresis_loss = active_calibration.hysteresis_loss,
    .hysteresis_loss_sensitivity = power_variance,
    .hysteresis_loss_angle = active_calibration.hysteresis_loss_angle,
    .vibration_fraction = active_calibration.vibration_fraction,
    .vibration_axis = active_calibration.vibration_axis,
  };
}

// Trying resilient backpropagation again, this time in the main loop.
// 
// We multiply the learning rates when we see consecutive updates with the same sign.
// And we decrease the mutliplier when the sign oscillates, this is called resilient
// backpropagation and it's relatively robust to the choice of initial learning rate
// but also really fast!
// 
// We can also declare that we have stable calibration parameters when the learning
// rate multiplier drops below a threshold.
constexpr float multiplier_minimum = 0.000'01f;
constexpr float multiplier_stable_threshold = 0.000'1f;
constexpr float multiplier_maximum = 10.f;
constexpr float multiplier_change = std::pow(2, -8);
constexpr float multiplier_increase_factor = 1.f + multiplier_change;
constexpr float multiplier_decrease_factor = 1.f - 2 * multiplier_change;


static inline void start_calibration_series() {
  // Clear the schedule of any pending states.
  if (drive_state_queue.read_available()) drive_state_queue.mark_all_read();

  drive_state_queue.write(hex_mini_drive::DriveState{
    .motor_flags = ENABLE_ALL_OUTPUTS,
    .closed_loop = false,
    .duration = calibration_duration,
    .target_value = calibration_instance.target_pwm,
    .angle_value = active_angle,
    .target_speed = calibration_instance.target_speed,
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

static inline hex_mini_drive::CalibrationInstance make_calibration_instance(hex_mini_drive::Calibration const& calibration){
  return hex_mini_drive::CalibrationInstance{
    .resistance_x = calibration.resistance,
    .resistance_y = calibration.resistance,
    .inductance = calibration.inductance,
    .inductance_saliency = calibration.inductance_saliency,
    .hysteresis_loss = calibration.hysteresis_loss,
    .vibration_fraction = calibration.vibration_fraction
  };
}

static inline void control_mode_calibrate(hex_mini_drive::Calibration const& active_calibration){
  control_mode_idle();
  calibration_instance = init_calibration_instance(
    make_calibration_instance(active_calibration), 
    calibration_target_increment, 
    calibration_speed_base
  );
  start_calibration_series();
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

      // Pre-emptively send the current calibration instance if calibrating.
      if (control_mode == ControlMode::CALIBRATE) queue_message(hex_mini_drive::Message{
        .message_code = hex_mini_drive::MessageCode::CALIBRATION_INSTANCE,
        .message_data = calibration_instance
      });
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
      // Run calibration routine initialized with the currently active calibration parameters.
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
// Return whether the parameter was valid (true if unchanged, false if changed).
static inline bool validate_parameter_update(float const& minimum_value, float & value, float const& value_change) {
  // Keep zeroes 0 and also zero out infinite/nan values.
  if (value == 0 or not std::isfinite(value)) {
    value = 0;
    return false;
  }
  
  const float new_value = value + value_change;

  if (new_value < minimum_value) {
    value = minimum_value;
    return false;
  } else {
    value = new_value;
    return true;
  }
}


static inline void finish_calibration_series(const bool remember_intermediate = false){
  queue_message(hex_mini_drive::Message{
    .message_code = hex_mini_drive::MessageCode::CALIBRATION_INSTANCE,
    .message_data = calibration_instance
  });

  const bool zero_speed = calibration_instance.target_speed == 0.f;
  
  float target = calibration_instance.target_pwm;
  // Go the the base frequency or multiply along an octave until the max frequency.
  float speed = (zero_speed ? calibration_speed_base : 
    (calibration_instance.target_speed * calibration_speed_multiplier)
  );

  if (speed > calibration_speed_max) {
    // Increment target pwm if we reach the end of the speed scanning.
    speed = calibration_speed_base;
    target += calibration_target_increment;

    // Finish calibration mode and return to idle when we have scanned through all target PWM values.
    if (target > calibration_target_max) return control_mode_idle();
  }


  // We have another series to run if we've reach this spot, set it up and go!
  calibration_instance = init_calibration_instance(
    // Carry over the current calibration otherwise default to active calibration.
    remember_intermediate ? calibration_instance : make_calibration_instance(calibration), 
    target, 
    speed
  );

  start_calibration_series();
}

static inline void calibrate_from_readout(
  hex_mini_drive::Readout const& readout, 
  const bool remember_intermediate = true, 
  const bool repeat_until_stable = true
) {
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

  // Check when we naturally finish the current calibration series by seeing when the drive state has finished.
  if (not drive_state_queue.read_available()) {
    // Check if we've actually started.
    if (calibration_instance.count == 0) {
      // Hmm, we haven't started this series yet; restart it.
      start_calibration_series();
      // Skip this readout since we just queued a new state.
    } else {
      if (repeat_until_stable) {
        queue_message(hex_mini_drive::Message{
          .message_code = hex_mini_drive::MessageCode::CALIBRATION_INSTANCE,
          .message_data = calibration_instance
        });
        // We are calibrating at speed but we have not stabilized, so... keep going I guess.
        start_calibration_series();
      } else {
        // We started so now that drive_state is done, we are naturally ending, without reaching stability.
        finish_calibration_series(false);
      }
    }
    // In either case we skip this readout.
    return;
  }

  // Stop calibration updates when we are undervoltage or overcurrent, we won't get accurate readings.
  const bool undervoltage = readout.vcc_voltage < control_parameters.vcc_undervoltage;
  const bool overcurrent = readout.current_magnitude > max_drive_current;

  // Abort the calibration instance if we triggered the stop conditions.
  if (undervoltage or overcurrent) {
    // We already have stats, abort calibration instance.
    if (calibration_instance.count) {
      drive_state_queue.mark_all_read();
      finish_calibration_series(false);
    }
    // In either case we skip this readout from calibration.
    return;
  }

  // Because our best source of voltage information is the drive_voltage we also need to make sure
  // we are driving with enough power to get any information.

  const float drive_voltage_square = square(readout.x_drive_voltage) + square(readout.y_drive_voltage);

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

  // Assuming the motor is oscillating with a roughly fixed angle, then... maybe...
  // the back emf from vibrations will act in opposing phase to the drive voltage
  // projected onto the emf axis of the rotor. The rotor, vibrating in place will
  // produce a vibrating emf response mostly on it's own axis.
  const float sin_vibration = get_sin(calibration_instance.vibration_axis);
  const float cos_vibration = get_cos(calibration_instance.vibration_axis);

  const float A_vibration = readout.x_current * cos_vibration + readout.y_current * sin_vibration;
  const float B_vibration = readout.x_current * sin_vibration - readout.y_current * cos_vibration;

  const float x_vibration = 0.5f * (readout.x_current + A_vibration);
  const float y_vibration = 0.5f * (readout.y_current + B_vibration);

  const float x_vibration_voltage = x_vibration * calibration_instance.vibration_fraction;
  const float y_vibration_voltage = y_vibration * calibration_instance.vibration_fraction;
  
  // Calculate the resistive voltage drop across the coil and MOSFET resistance.
  const float x_resistive_voltage = readout.x_current * calibration_instance.resistance_x;
  const float y_resistive_voltage = readout.y_current * calibration_instance.resistance_y;

  // And the current diffs for calculating inductance.
  const float x_inductance = readout.x_current_diff * current_diff_to_voltage_units;
  const float y_inductance = readout.y_current_diff * current_diff_to_voltage_units;

  // Note: The saliency effect is due to the iron between magnets so it occurs twice per 
  // electrical revolution, once for each pole. Thus we must use the doubled rotor angle.

  const float sin_saliency = get_sin(calibration_instance.saliency_angle);
  const float cos_saliency = get_cos(calibration_instance.saliency_angle);

  const float x_inductance_saliency = (
    -x_inductance * cos_saliency 
    -y_inductance * sin_saliency
  );
  
  const float y_inductance_saliency = (
    -x_inductance * sin_saliency
    +y_inductance * cos_saliency
  );

  const float x_inductance_saliency_voltage = x_inductance_saliency * calibration_instance.inductance_saliency;
  const float y_inductance_saliency_voltage = y_inductance_saliency * calibration_instance.inductance_saliency;

  const float x_inductance_base_voltage = x_inductance * calibration_instance.inductance;
  const float y_inductance_base_voltage = y_inductance * calibration_instance.inductance;

  const float x_inductance_voltage = x_inductance_base_voltage + x_inductance_saliency_voltage;
  const float y_inductance_voltage = y_inductance_base_voltage + y_inductance_saliency_voltage;
  
  const float sin_hysteresis_loss = get_sin(calibration_instance.hysteresis_loss_angle);
  const float cos_hysteresis_loss = get_cos(calibration_instance.hysteresis_loss_angle);

  const float A_hysteresis_loss = (
    readout.x_current * x_inductance_voltage - 
    readout.y_current * y_inductance_voltage
  ) * voltage_mul_current_to_power;

  const float B_hysteresis_loss = (
    readout.x_current * y_inductance_voltage + 
    readout.y_current * x_inductance_voltage
  ) * voltage_mul_current_to_power;

  const float x_hysteresis_loss = A_hysteresis_loss * cos_hysteresis_loss - B_hysteresis_loss * sin_hysteresis_loss;
  const float y_hysteresis_loss = A_hysteresis_loss * sin_hysteresis_loss + B_hysteresis_loss * cos_hysteresis_loss;

  const float x_hysteresis_loss_voltage = calibration_instance.hysteresis_loss * x_hysteresis_loss;
  
  const float y_hysteresis_loss_voltage = calibration_instance.hysteresis_loss * y_hysteresis_loss;

  // Calibrate without EMF!!!
  // ------------------------
  // 
  // It's very important that we don't calibrate while the motor is rotating and generating EMF.
  // During sensorless running we infer the EMF as the error of the electrical equation, but
  // that means we can't use the inferred EMF to calibrate the electrical parameters accurately.
  // Especially because the resistance voltage and EMF voltage are covarying quantities.
  // Inductance and the saliency bias are also highly sensitive to any lag in the EMF estimation.

  const float x_voltage_error = (
    x_inductance_voltage + 
    x_resistive_voltage + 
    x_hysteresis_loss_voltage + 
    x_vibration_voltage + 
    -readout.x_drive_voltage
  );
  const float y_voltage_error = (
    y_inductance_voltage + 
    y_resistive_voltage + 
    y_hysteresis_loss_voltage + 
    y_vibration_voltage + 
    -readout.y_drive_voltage
  );

  // Defining the loss as the square(x_voltage_error) + square(y_voltage_error) we get the derivative gradients with 
  // respect to each of the parameters we're tracking. We only care about the sign so we should be careful to skip updates when 
  // there isn't enough energy in the error to warrant an update.
  const float square_voltage_error = square(x_voltage_error) + square(y_voltage_error);

  const float square_current = square(readout.current_magnitude);

  const float square_inductance = square(x_inductance) + square(y_inductance);


  // Store stats
  // -----------
  // 
  // The error is evolving during the optimization, we want to store the end-ish value after we've optimized our parameters.

  const float drive_voltage = readout.vcc_voltage * readout.active_pwm * active_pwm_to_voltage_conversion;
  
  calibration_instance.avg_drive_voltage += (drive_voltage - calibration_instance.avg_drive_voltage) * calibration_stats_ki;
  calibration_instance.avg_current_magnitude += (readout.current_magnitude - calibration_instance.avg_current_magnitude) * calibration_stats_ki;

  const float square_error_change = square_voltage_error - calibration_instance.avg_square_error;
  calibration_instance.avg_square_error += square_error_change * calibration_stats_ki;
  
  // Gradients & Parameter Updates
  // -----------------------------
  // 
  // Compute the gradients of the loss with respect to each of the tracked parameters.
  // 
  // ! We must not condition the updates apparently...

  // Count how often we see enough current to meaningfully calibrate resistance (we still have to do it, but we mark it as uncertain).
  const bool resistance_excited = square_current > current_variance;
  calibration_instance.resistance_excitation_fraction += (resistance_excited - calibration_instance.resistance_excitation_fraction) * calibration_stats_ki;

  // Also track inductance excitation.
  const bool inductance_excited = square_inductance > inductance_variance;
  calibration_instance.inductance_excitation_fraction += (inductance_excited - calibration_instance.inductance_excitation_fraction) * calibration_stats_ki;
  
  // Compute the group learning rate multiplier based on the sign of the change of our average error.
  // We want to multiply the learning rate when the average error consistenly decreases and we want to
  // lower the learning rate the error oscillates.
  if(calibration_instance.learning_rate_resilience_sign * square_error_change < 0) {
    // Swap the sign when the gradient is counter to the previous one, we're converging.
    calibration_instance.learning_rate_multiplier = max(calibration_instance.learning_rate_multiplier * multiplier_decrease_factor, multiplier_minimum);
    calibration_instance.learning_rate_resilience_sign = -calibration_instance.learning_rate_resilience_sign;
  } else {
    // Keep the sign when the gradient is consistent with the previous one, we're still moving in the right direction.
    if (square_error_change < 0) {
      calibration_instance.learning_rate_multiplier = min(calibration_instance.learning_rate_multiplier * multiplier_increase_factor, multiplier_maximum);
    }
  }


  calibration_instance.resistance_sensitivity = square_current + current_variance;
  const float resistance_scaling = 1.f / calibration_instance.resistance_sensitivity;

  const float resistance_x_gradient = resistance_scaling * x_voltage_error * readout.x_current;
  const float resistance_y_gradient = resistance_scaling * y_voltage_error * readout.y_current;
  
  validate_parameter_update(
    minimum_resistance, 
    calibration_instance.resistance_x,
    -resistance_x_gradient * control_parameters.resistance_ki * calibration_instance.learning_rate_multiplier
  );
  validate_parameter_update(
    minimum_resistance, 
    calibration_instance.resistance_y,
    -resistance_y_gradient * control_parameters.resistance_ki * calibration_instance.learning_rate_multiplier
  );

  calibration_instance.inductance_sensitivity = square_inductance + inductance_variance;
  const float inductance_scaling = 1.f / calibration_instance.inductance_sensitivity;

  const float inductance_gradient = inductance_scaling * (
    x_voltage_error * x_inductance +
    y_voltage_error * y_inductance
  );
  
  // Keep 0 inductance exactly 0 (meaning disabled), but otherwise make sure it's minimum positive value.
  validate_parameter_update(
    minimum_inductance, 
    calibration_instance.inductance, 
    -inductance_gradient * control_parameters.inductance_ki * calibration_instance.learning_rate_multiplier
  );

  const float inductance_saliency_gradient = inductance_scaling * (
    x_voltage_error * x_inductance_saliency +
    y_voltage_error * y_inductance_saliency
  );


  // It was valid if above minimum, otherwise it got capped before going negative. This value is mediated by a directional
  // angle that we are also optimizing, it will go negative if we are stuck at 180 degree flip. In that case we can escape
  // the false equilibrium quickly by flipping the angle (and resetting the resilience multiplier).
  if (validate_parameter_update(
    minimum_inductance_saliency,
    calibration_instance.inductance_saliency, 
    -inductance_saliency_gradient * control_parameters.inductance_saliency_ki * calibration_instance.learning_rate_multiplier
  )) {
    // If we're normal condition, then just gradient descent the angle too.
    const float saliency_angle_gradient = inductance_scaling * (
      x_voltage_error * -y_inductance_saliency +
      y_voltage_error * x_inductance_saliency
    ) * radians_to_angle_units / calibration_instance.inductance_saliency;

    calibration_instance.saliency_angle -= static_cast<int32_t>(saliency_angle_gradient * control_parameters.saliency_angle_ki);
  } else {
    // If we went negative/below minimum it's likely the angle converged to the opposite pole. Flip and reset the angle state.
    calibration_instance.saliency_angle += half_circle;
  }

  calibration_instance.hysteresis_loss_sensitivity = square(x_hysteresis_loss) + square(y_hysteresis_loss) + power_variance;
  
  const float hysteresis_loss_scaling = 1.f / calibration_instance.hysteresis_loss_sensitivity;

  // And gradients for micro rotor vibrations.
  const float hysteresis_loss_gradient = hysteresis_loss_scaling * (
    x_voltage_error * x_hysteresis_loss + 
    y_voltage_error * y_hysteresis_loss
  );

  if(validate_parameter_update(
    minimum_hysteresis_loss,
    calibration_instance.hysteresis_loss,
    -hysteresis_loss_gradient * control_parameters.hysteresis_loss_ki * calibration_instance.learning_rate_multiplier
  )){
    const float hysteresis_loss_angle_gradient = hysteresis_loss_scaling * (
      x_voltage_error * -y_hysteresis_loss + 
      y_voltage_error * x_hysteresis_loss
    ) * radians_to_angle_units / calibration_instance.hysteresis_loss;

    calibration_instance.hysteresis_loss_angle -= static_cast<int32_t>(hysteresis_loss_angle_gradient * control_parameters.hysteresis_loss_angle_ki);
  } else {
    calibration_instance.hysteresis_loss_angle += half_circle;
  }

  const float vibration_factor_gradient = resistance_scaling * (
    x_voltage_error * x_vibration + 
    y_voltage_error * y_vibration
  );

  if (validate_parameter_update(
    minimum_vibration_fraction,
    calibration_instance.vibration_fraction,
    -vibration_factor_gradient * control_parameters.vibration_fraction_ki * calibration_instance.learning_rate_multiplier
  )) {
    const float vibration_axis_gradient = resistance_scaling * (
      x_voltage_error * -B_vibration + 
      y_voltage_error * A_vibration
    ) * radians_to_angle_units / calibration_instance.vibration_fraction;
    calibration_instance.vibration_axis -= static_cast<int32_t>(vibration_axis_gradient * control_parameters.vibration_axis_ki);
  } else {
    // Not sure it's correct to flip vibration axis by 180 degrees.
  }


  // Check for stability
  // -------------------

  // Periodically check if we stablizied and move on.
  if (calibration_instance.count % calibration_stability_check_period == 0) {
    // const bool unexcited_resistance = calibration_instance.resistance_excitation_fraction < excitation_fraction_threshold;
    // const bool unexcited_inductance = calibration_instance.inductance_excitation_fraction < excitation_fraction_threshold;
    // const bool zero_speed = calibration_instance.target_speed == 0.f;

    if (calibration_instance.learning_rate_multiplier < multiplier_stable_threshold) finish_calibration_series(remember_intermediate);
  }
}

static inline void process_readout(hex_mini_drive::Readout const& readout) {
  // Count if we skip any readouts.
  // const uint16_t readouts_since_last_tick = (readout_number_base + readout.readout_number - last_readout_number) % readout_number_base;

  last_readout_number = readout.readout_number;

  angle = readout.angle;
  active_angle = readout.active_angle;

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
      {readout.x_current, readout.y_current}, 
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