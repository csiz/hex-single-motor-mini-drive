// API interface definition for the Hex Single Motor Mini Drive.
// 
// The first 2 bytes of each message contain the message code. The remainder 
// of the message contains data as defined below, writting the fields in order.
// as their bit representation, little-endian.

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#include "byte_handling.hpp"

// Constants
// ---------

constexpr uint16_t HISTORY_SIZE = 336;

// Message Codes
enum MessageCode : uint16_t {
  NullCommand = 0,
  Readout = 8224,
  StreamFullReadouts = 8225,
  GetReadoutsSnapshot = 8226,
  FullReadout = 8227,
  SetStateOff = 8240,
  SetStateDrive6Sector = 8241,
  SetStateTestAllPermutations = 8242,
  SetStateFreewheel = 8244,
  SetStateTestGroundShort = 8246,
  SetStateTestPositiveShort = 8247,
  SetStateTestUDirections = 8249,
  SetStateTestUIncreasing = 8250,
  SetStateTestUDecreasing = 8251,
  SetStateTestVIncreasing = 8252,
  SetStateTestVDecreasing = 8253,
  SetStateTestWIncreasing = 8254,
  SetStateTestWDecreasing = 8255,
  SetStateHoldUPositive = 12320,
  SetStateHoldVPositive = 12321,
  SetStateHoldWPositive = 12322,
  SetStateHoldUNegative = 12323,
  SetStateHoldVNegative = 12324,
  SetStateHoldWNegative = 12325,
  SetStateDrivePeriodic = 12352,
  SetStateDriveSmooth = 16432,
  SetStateDriveTorque = 16433,
  SetStateDriveBatteryPower = 16434,
  SetStateDriveSpeed = 16435,
  SetStateSeekAngleWithPower = 16436,
  SetStateSeekAngleWithTorque = 16437,
  SetStateSeekAngleWithSpeed = 16438,
  CurrentCalibration = 16448,
  GetCurrentCalibration = 16449,
  SetCurrentCalibration = 16450,
  ResetCurrentCalibration = 16451,
  HallPositions = 16452,
  GetHallPositions = 16453,
  SetHallPositions = 16454,
  ResetHallPositions = 16455,
  ControlParameters = 16457,
  SetControlParameters = 16458,
  GetControlParameters = 16459,
  ResetControlParameters = 16460,
  SetAngle = 16464,
  SaveSettingsToFlash = 16512,
  UnitTestOutput = 20544,
  RunUnitTestFunkyAtan = 20546,
  RunUnitTestFunkyAtanPart2 = 20547,
  RunUnitTestFunkyAtanPart3 = 20548
};

// Message Structures
// ------------------

// Basic readout of the motor driver internal state; can be recorded contiguously
// into a history buffer after a commanded event.
struct Readout {

  static constexpr uint16_t message_code = 8224;
  static constexpr size_t message_size = 34;
  
  // The PWM commands for the motor outputs; concatenated into a single value.
  uint32_t pwm_commands;
  
  // Readout number; used to identify the readout in the history.
  uint16_t readout_number;
  
  // Driver state flags; packed into a single 16-bit value.
  uint16_t state_flags;
  
  // Raw phase U current readout (ADC value).
  int16_t u_current;
  
  // Raw phase V current readout (ADC value).
  int16_t v_current;
  
  // Raw phase W current readout (ADC value).
  int16_t w_current;
  
  // Raw reference readout (ADC value); this is the reference voltage for the current 
  // readouts as seen by the amplifier. Needs to be subtracted from the phase readouts.
  int16_t ref_readout;
  
  // Phase U current readout difference to previous readout.
  int16_t u_current_diff;
  
  // Phase V current readout difference to previous readout.
  int16_t v_current_diff;
  
  // Phase W current readout difference to previous readout.
  int16_t w_current_diff;
  
  // Best estimate for the rotor magnetic angle.
  int16_t angle;
  
  // Error of the angle measured from EMF to the rotor angle prediction.
  int16_t angle_adjustment;
  
  // Best estimate for the rotor magnetic angular speed.
  int16_t angular_speed;
  
  // Instantaneous VCC voltage readout (ADC value); from resistance divider.
  int16_t vcc_voltage;
  
  // EMF voltage magnitude. The EMF is always along the beta direction, but we can have 
  // errors in the measurements and the rotor position and thus we see alpha component 
  // as well. We can rotate the EMF voltage vector fully to the beta direction and get 
  // closer to the actual EMF voltage magnitude.
  int16_t emf_voltage_magnitude;
  
};


// Complete readout of the motor driver internal state for exploration and data stream.
struct FullReadout : Readout {

  static constexpr uint16_t message_code = 8227;
  static constexpr size_t message_size = 82;
  
  // Tick rate; the number of main loop (communication and commands) updates per second.
  uint16_t main_loop_rate;
  
  // ADC update rate; the number of ADC readouts per second. This is usually higher 
  // than the main loop rate because we read the ADCs multiple times per main loop.
  uint16_t adc_update_rate;
  
  // Instantaneous temperature readout (ADC value); from the temperature sensor.
  uint16_t temperature;
  
  // Current maximum PWM allowed by the driver.
  uint16_t live_max_pwm;
  
  // PWM counter value at the start of the control update. Should occur immediately 
  // after the halfway point.
  int16_t cycle_start_tick;
  
  // PWM counter value at the end of the control update. Should occur immediately 
  // before the halfway point.
  int16_t cycle_end_tick;
  
  // Current in DQ0 coordinates; aligned with the rotor angle.
  int16_t direct_current;
  
  // Current in DQ0 coordinates; crossed with the rotor angle.
  int16_t quadrature_current;
  
  // EMF voltage in DQ0 coordinates; aligned with the rotor angle.
  int16_t direct_emf_voltage;
  
  // EMF voltage in DQ0 coordinates; crossed with the rotor angle.
  int16_t quadrature_emf_voltage;
  
  // Total power used/given to VCC line (the battery usually).
  int16_t total_power;
  
  // Resistive power; the power dissipated in the phase resistances.
  int16_t resistive_power;
  
  // EMF power; the power used to drive the motor (which is reflected to the 
  // inductors as back EMF).
  int16_t emf_power;
  
  // Inductive power; the power pushed into the inductor magnetic fields.
  int16_t inductive_power;
  
  // Motor constant; a measure of how strong the motor is. It is computed as the 
  // ratio between the quadrature EMF voltage and the angular speed.
  int16_t motor_constant;
  
  // The current angle.
  int16_t inductor_angle;
  
  // The measured acceleration of the rotor.
  int16_t rotor_acceleration;
  
  // Integrated number of EMF deduced rotor angle rotations since startup.
  int16_t rotations;
  
  // Magnitude of the phase current in the DQ0 coordinate frame.
  int16_t current_magnitude;
  
  // Variance of the EMF angle error; used to determine if the EMF angle is too noisy to update.
  int16_t emf_angle_error_variance;
  
  // Lead angle for the motor driving; used to adjust the phase voltages to drive the 
  // motor efficiently.
  int16_t lead_angle;
  
  // Target PWM value for the motor outputs, value set by the advanced control algorithms.
  int16_t target_pwm;
  
  // Target for the advanced control algorithms.
  int16_t secondary_target;
  
  // Spare debug output.
  int16_t seek_integral;
  
};


// Calibration factors for the current sensors.
// 
// The soldering joints vary during manufacturing and therefore they affect the total
// resistance of the shunt resistors. We can calibrate for this effect by multiplying
// the readout by a factor for each phase.
// 
// For the v1 design, we shall improve the shunt resistors and soldering pad design to
// to improve the accuracy. We can then switch to automatically calibrating the phase
// resistance and motor inductance. For now we calibrate using the motor monitor app.
struct CurrentCalibration {

  static constexpr uint16_t message_code = 16448;
  static constexpr size_t message_size = 10;
  
  // Adjustment factor for the U phase current readout.
  int16_t u_factor;
  
  // Adjustment factor for the V phase current readout.
  int16_t v_factor;
  
  // Adjustment factor for the W phase current readout.
  int16_t w_factor;
  
  // Adjustment factor for the motor inductance; used to calibrate the coil inductance.
  int16_t inductance_factor;
  
};


// Hall sensor position calibration data.
// 
// Apparently, millimiter precision in the placement of the hall sensor chips means an error up to 
// 30 degrees in the electrical angle of the magnetic rotor. Note that for each physical rotation
// of the magnet there are N magnet poles times P coil pairs rotations of the electrical angle.
// 
// With this big of an error, we need to calibrate the hall sensor positions using the angle
// inferred from the back EMF voltage induced in the coils.
struct HallPositions {

  static constexpr uint16_t message_code = 16452;
  static constexpr size_t message_size = 74;
  
  // The angle at the transition to the current sector from the left and from the 
  // right. By "left" I mean the hall sector has transitioned from a lower to a higher 
  // number, the rotor has a positive speed and is rotating counter-clockwise (trigonometric 
  // direction). The left angle is lower than the right angle. Note that the left angle 
  // of a sector and the right angle of the previous sector do not coincide because the 
  // hall sensors have a designed hysteresis that latches the output.
  std::array<std::array<uint16_t, 2>, 6> sector_transition_angles;
  
  // The variance of the angles (it is expensive to compute the standard deviation with 
  // a square root but we only need the variance, so we only store the variance).
  std::array<std::array<uint16_t, 2>, 6> sector_transition_variances;
  
  // The center angle of each sector; the average of the left and right angles.
  std::array<uint16_t, 6> sector_center_angles;
  
  // The variance of the center angles; at the moment it represents the span of the hall sector.
  std::array<uint16_t, 6> sector_center_variances;
  
};


// Parameters used in the motor control loop; for detailed descriptions check the
// motor monitor page. It's useful to modify the values and inspect the changes to
// the respective variables in the readout while driving a physical motor.
struct ControlParameters {

  static constexpr uint16_t message_code = 16457;
  static constexpr size_t message_size = 78;
  
  // Magnet position integral gain.
  int16_t rotor_angle_ki;
  
  // Magnet angular speed integral gain.
  int16_t rotor_angular_speed_ki;
  
  // Averaging gain for the acceleration of the rotor.
  int16_t rotor_acceleration_ki;
  
  // Motor constant integral gain.
  int16_t motor_constant_ki;
  
  // Sign of the motor direction (positive by default, negative to reverse turning direction).
  int16_t motor_direction;
  
  // Number of incorrect direction detections before we flip our motor angle.
  int16_t incorrect_direction_threshold;
  
  // Maximum PWM adjustment per cycle.
  int16_t max_pwm_change;
  
  // Maximum target angle change per cycle.
  int16_t max_angle_change;
  
  // Minimum EMF voltage to consider EMF detected (above the noise level)
  int16_t min_emf_voltage;
  
  // Integral gain for the hall angle adjustment (0 to ignore).
  int16_t hall_angle_ki;
  
  // Lead angle integral gain for efficient driving.
  int16_t lead_angle_control_ki;
  
  // Torque control gain.
  int16_t torque_control_ki;
  
  // Battery power control gain.
  int16_t battery_power_control_ki;
  
  // Speed control gain.
  int16_t speed_control_ki;
  
  // Probing angular speed for initial EMF detection.
  int16_t probing_angular_speed;
  
  // Maximum PWM difference from motor PWM required to compensate back EMF.
  int16_t max_pwm_difference;
  
  // Maximum EMF angle correction variance when it's too noisy to update the angle.
  int16_t emf_angle_error_variance_threshold;
  
  // Minium EMF voltage to compute the motor constant.
  int16_t min_emf_for_motor_constant;
  
  // Maximum resistive power that can be dissipated in the motor coils.
  int16_t max_resistive_power;
  
  // Resistive power long duration average observer gain.
  int16_t resistive_power_ki;
  
  // Maximum angular speed of the motor.
  int16_t max_angular_speed;
  
  // Maximum power draw from the battery (proxy for maximum current).
  int16_t max_power_draw;
  
  // Power draw long duration average observer gain.
  int16_t power_draw_ki;
  
  // Maximum PWM value for the motor outputs.
  int16_t max_pwm;
  
  // Seek via torque, prediction duration factor for integral error.
  int16_t seek_via_torque_k_prediction;
  
  // Seek via torque, integral gain for the PID control.
  int16_t seek_via_torque_ki;
  
  // Seek via torque, proportional gain for the PID control.
  int16_t seek_via_torque_kp;
  
  // Seek via torque, derivative gain for the PID control.
  int16_t seek_via_torque_kd;
  
  // Seek via power, prediction duration factor for integral error.
  int16_t seek_via_power_k_prediction;
  
  // Seek via power, integral gain for the PID control.
  int16_t seek_via_power_ki;
  
  // Seek via power, proportional gain for the PID control.
  int16_t seek_via_power_kp;
  
  // Seek via power, derivative gain for the PID control.
  int16_t seek_via_power_kd;
  
  // Seek via speed, prediction duration factor for integral error.
  int16_t seek_via_speed_k_prediction;
  
  // Seek via speed, integral gain for the PID control.
  int16_t seek_via_speed_ki;
  
  // Seek via speed, proportional gain for the PID control.
  int16_t seek_via_speed_kp;
  
  // Seek via speed, derivative gain for the PID control.
  int16_t seek_via_speed_kd;
  
  // Resistance of motor coils per phase (star configuration).
  int16_t phase_resistance;
  
  // Inductance of motor coils per phase (star configuration).
  int16_t phase_inductance;
  
};


struct UnitTestOutput {

  static constexpr uint16_t message_code = 20544;
  static constexpr size_t message_size = 258;
  
  std::array<uint8_t, 256> data;
  
};


// Serialization Functions
// -----------------------

static inline bool serialise(Readout const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, Readout::message_code);
  offset += 2;
  
  write_uint32(buffer + offset, value.pwm_commands);
  offset += 4;
  write_uint16(buffer + offset, value.readout_number);
  offset += 2;
  write_uint16(buffer + offset, value.state_flags);
  offset += 2;
  write_int16(buffer + offset, value.u_current);
  offset += 2;
  write_int16(buffer + offset, value.v_current);
  offset += 2;
  write_int16(buffer + offset, value.w_current);
  offset += 2;
  write_int16(buffer + offset, value.ref_readout);
  offset += 2;
  write_int16(buffer + offset, value.u_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.v_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.w_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.angle);
  offset += 2;
  write_int16(buffer + offset, value.angle_adjustment);
  offset += 2;
  write_int16(buffer + offset, value.angular_speed);
  offset += 2;
  write_int16(buffer + offset, value.vcc_voltage);
  offset += 2;
  write_int16(buffer + offset, value.emf_voltage_magnitude);
  offset += 2;
  return true;
}

static inline bool deserialise(Readout & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != Readout::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != Readout::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  result.pwm_commands = read_uint32(buffer + offset);
  offset += 4;
  result.readout_number = read_uint16(buffer + offset);
  offset += 2;
  result.state_flags = read_uint16(buffer + offset);
  offset += 2;
  result.u_current = read_int16(buffer + offset);
  offset += 2;
  result.v_current = read_int16(buffer + offset);
  offset += 2;
  result.w_current = read_int16(buffer + offset);
  offset += 2;
  result.ref_readout = read_int16(buffer + offset);
  offset += 2;
  result.u_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.v_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.w_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.angle = read_int16(buffer + offset);
  offset += 2;
  result.angle_adjustment = read_int16(buffer + offset);
  offset += 2;
  result.angular_speed = read_int16(buffer + offset);
  offset += 2;
  result.vcc_voltage = read_int16(buffer + offset);
  offset += 2;
  result.emf_voltage_magnitude = read_int16(buffer + offset);
  offset += 2;
  return true;
}

static inline bool serialise(FullReadout const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, FullReadout::message_code);
  offset += 2;
  
  // Fields from Readout
  write_uint32(buffer + offset, value.pwm_commands);
  offset += 4;
  write_uint16(buffer + offset, value.readout_number);
  offset += 2;
  write_uint16(buffer + offset, value.state_flags);
  offset += 2;
  write_int16(buffer + offset, value.u_current);
  offset += 2;
  write_int16(buffer + offset, value.v_current);
  offset += 2;
  write_int16(buffer + offset, value.w_current);
  offset += 2;
  write_int16(buffer + offset, value.ref_readout);
  offset += 2;
  write_int16(buffer + offset, value.u_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.v_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.w_current_diff);
  offset += 2;
  write_int16(buffer + offset, value.angle);
  offset += 2;
  write_int16(buffer + offset, value.angle_adjustment);
  offset += 2;
  write_int16(buffer + offset, value.angular_speed);
  offset += 2;
  write_int16(buffer + offset, value.vcc_voltage);
  offset += 2;
  write_int16(buffer + offset, value.emf_voltage_magnitude);
  offset += 2;
  
  write_uint16(buffer + offset, value.main_loop_rate);
  offset += 2;
  write_uint16(buffer + offset, value.adc_update_rate);
  offset += 2;
  write_uint16(buffer + offset, value.temperature);
  offset += 2;
  write_uint16(buffer + offset, value.live_max_pwm);
  offset += 2;
  write_int16(buffer + offset, value.cycle_start_tick);
  offset += 2;
  write_int16(buffer + offset, value.cycle_end_tick);
  offset += 2;
  write_int16(buffer + offset, value.direct_current);
  offset += 2;
  write_int16(buffer + offset, value.quadrature_current);
  offset += 2;
  write_int16(buffer + offset, value.direct_emf_voltage);
  offset += 2;
  write_int16(buffer + offset, value.quadrature_emf_voltage);
  offset += 2;
  write_int16(buffer + offset, value.total_power);
  offset += 2;
  write_int16(buffer + offset, value.resistive_power);
  offset += 2;
  write_int16(buffer + offset, value.emf_power);
  offset += 2;
  write_int16(buffer + offset, value.inductive_power);
  offset += 2;
  write_int16(buffer + offset, value.motor_constant);
  offset += 2;
  write_int16(buffer + offset, value.inductor_angle);
  offset += 2;
  write_int16(buffer + offset, value.rotor_acceleration);
  offset += 2;
  write_int16(buffer + offset, value.rotations);
  offset += 2;
  write_int16(buffer + offset, value.current_magnitude);
  offset += 2;
  write_int16(buffer + offset, value.emf_angle_error_variance);
  offset += 2;
  write_int16(buffer + offset, value.lead_angle);
  offset += 2;
  write_int16(buffer + offset, value.target_pwm);
  offset += 2;
  write_int16(buffer + offset, value.secondary_target);
  offset += 2;
  write_int16(buffer + offset, value.seek_integral);
  offset += 2;
  return true;
}

static inline bool deserialise(FullReadout & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != FullReadout::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != FullReadout::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  // Fields from Readout
  result.pwm_commands = read_uint32(buffer + offset);
  offset += 4;
  result.readout_number = read_uint16(buffer + offset);
  offset += 2;
  result.state_flags = read_uint16(buffer + offset);
  offset += 2;
  result.u_current = read_int16(buffer + offset);
  offset += 2;
  result.v_current = read_int16(buffer + offset);
  offset += 2;
  result.w_current = read_int16(buffer + offset);
  offset += 2;
  result.ref_readout = read_int16(buffer + offset);
  offset += 2;
  result.u_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.v_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.w_current_diff = read_int16(buffer + offset);
  offset += 2;
  result.angle = read_int16(buffer + offset);
  offset += 2;
  result.angle_adjustment = read_int16(buffer + offset);
  offset += 2;
  result.angular_speed = read_int16(buffer + offset);
  offset += 2;
  result.vcc_voltage = read_int16(buffer + offset);
  offset += 2;
  result.emf_voltage_magnitude = read_int16(buffer + offset);
  offset += 2;
  
  result.main_loop_rate = read_uint16(buffer + offset);
  offset += 2;
  result.adc_update_rate = read_uint16(buffer + offset);
  offset += 2;
  result.temperature = read_uint16(buffer + offset);
  offset += 2;
  result.live_max_pwm = read_uint16(buffer + offset);
  offset += 2;
  result.cycle_start_tick = read_int16(buffer + offset);
  offset += 2;
  result.cycle_end_tick = read_int16(buffer + offset);
  offset += 2;
  result.direct_current = read_int16(buffer + offset);
  offset += 2;
  result.quadrature_current = read_int16(buffer + offset);
  offset += 2;
  result.direct_emf_voltage = read_int16(buffer + offset);
  offset += 2;
  result.quadrature_emf_voltage = read_int16(buffer + offset);
  offset += 2;
  result.total_power = read_int16(buffer + offset);
  offset += 2;
  result.resistive_power = read_int16(buffer + offset);
  offset += 2;
  result.emf_power = read_int16(buffer + offset);
  offset += 2;
  result.inductive_power = read_int16(buffer + offset);
  offset += 2;
  result.motor_constant = read_int16(buffer + offset);
  offset += 2;
  result.inductor_angle = read_int16(buffer + offset);
  offset += 2;
  result.rotor_acceleration = read_int16(buffer + offset);
  offset += 2;
  result.rotations = read_int16(buffer + offset);
  offset += 2;
  result.current_magnitude = read_int16(buffer + offset);
  offset += 2;
  result.emf_angle_error_variance = read_int16(buffer + offset);
  offset += 2;
  result.lead_angle = read_int16(buffer + offset);
  offset += 2;
  result.target_pwm = read_int16(buffer + offset);
  offset += 2;
  result.secondary_target = read_int16(buffer + offset);
  offset += 2;
  result.seek_integral = read_int16(buffer + offset);
  offset += 2;
  return true;
}

static inline bool serialise(CurrentCalibration const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, CurrentCalibration::message_code);
  offset += 2;
  
  write_int16(buffer + offset, value.u_factor);
  offset += 2;
  write_int16(buffer + offset, value.v_factor);
  offset += 2;
  write_int16(buffer + offset, value.w_factor);
  offset += 2;
  write_int16(buffer + offset, value.inductance_factor);
  offset += 2;
  return true;
}

static inline bool deserialise(CurrentCalibration & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != CurrentCalibration::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != CurrentCalibration::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  result.u_factor = read_int16(buffer + offset);
  offset += 2;
  result.v_factor = read_int16(buffer + offset);
  offset += 2;
  result.w_factor = read_int16(buffer + offset);
  offset += 2;
  result.inductance_factor = read_int16(buffer + offset);
  offset += 2;
  return true;
}

static inline bool serialise(HallPositions const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, HallPositions::message_code);
  offset += 2;
  
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      write_uint16(buffer + offset, value.sector_transition_angles[i][j]);
      offset += 2;
    }
  }
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      write_uint16(buffer + offset, value.sector_transition_variances[i][j]);
      offset += 2;
    }
  }
  for (size_t i = 0; i < 6; ++i) {
    write_uint16(buffer + offset, value.sector_center_angles[i]);
    offset += 2;
  }
  for (size_t i = 0; i < 6; ++i) {
    write_uint16(buffer + offset, value.sector_center_variances[i]);
    offset += 2;
  }
  return true;
}

static inline bool deserialise(HallPositions & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != HallPositions::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != HallPositions::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      result.sector_transition_angles[i][j] = read_uint16(buffer + offset);
      offset += 2;
    }
  }
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      result.sector_transition_variances[i][j] = read_uint16(buffer + offset);
      offset += 2;
    }
  }
  for (size_t i = 0; i < 6; ++i) {
    result.sector_center_angles[i] = read_uint16(buffer + offset);
    offset += 2;
  }
  for (size_t i = 0; i < 6; ++i) {
    result.sector_center_variances[i] = read_uint16(buffer + offset);
    offset += 2;
  }
  return true;
}

static inline bool serialise(ControlParameters const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, ControlParameters::message_code);
  offset += 2;
  
  write_int16(buffer + offset, value.rotor_angle_ki);
  offset += 2;
  write_int16(buffer + offset, value.rotor_angular_speed_ki);
  offset += 2;
  write_int16(buffer + offset, value.rotor_acceleration_ki);
  offset += 2;
  write_int16(buffer + offset, value.motor_constant_ki);
  offset += 2;
  write_int16(buffer + offset, value.motor_direction);
  offset += 2;
  write_int16(buffer + offset, value.incorrect_direction_threshold);
  offset += 2;
  write_int16(buffer + offset, value.max_pwm_change);
  offset += 2;
  write_int16(buffer + offset, value.max_angle_change);
  offset += 2;
  write_int16(buffer + offset, value.min_emf_voltage);
  offset += 2;
  write_int16(buffer + offset, value.hall_angle_ki);
  offset += 2;
  write_int16(buffer + offset, value.lead_angle_control_ki);
  offset += 2;
  write_int16(buffer + offset, value.torque_control_ki);
  offset += 2;
  write_int16(buffer + offset, value.battery_power_control_ki);
  offset += 2;
  write_int16(buffer + offset, value.speed_control_ki);
  offset += 2;
  write_int16(buffer + offset, value.probing_angular_speed);
  offset += 2;
  write_int16(buffer + offset, value.max_pwm_difference);
  offset += 2;
  write_int16(buffer + offset, value.emf_angle_error_variance_threshold);
  offset += 2;
  write_int16(buffer + offset, value.min_emf_for_motor_constant);
  offset += 2;
  write_int16(buffer + offset, value.max_resistive_power);
  offset += 2;
  write_int16(buffer + offset, value.resistive_power_ki);
  offset += 2;
  write_int16(buffer + offset, value.max_angular_speed);
  offset += 2;
  write_int16(buffer + offset, value.max_power_draw);
  offset += 2;
  write_int16(buffer + offset, value.power_draw_ki);
  offset += 2;
  write_int16(buffer + offset, value.max_pwm);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_torque_k_prediction);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_torque_ki);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_torque_kp);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_torque_kd);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_power_k_prediction);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_power_ki);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_power_kp);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_power_kd);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_speed_k_prediction);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_speed_ki);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_speed_kp);
  offset += 2;
  write_int16(buffer + offset, value.seek_via_speed_kd);
  offset += 2;
  write_int16(buffer + offset, value.phase_resistance);
  offset += 2;
  write_int16(buffer + offset, value.phase_inductance);
  offset += 2;
  return true;
}

static inline bool deserialise(ControlParameters & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != ControlParameters::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != ControlParameters::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  result.rotor_angle_ki = read_int16(buffer + offset);
  offset += 2;
  result.rotor_angular_speed_ki = read_int16(buffer + offset);
  offset += 2;
  result.rotor_acceleration_ki = read_int16(buffer + offset);
  offset += 2;
  result.motor_constant_ki = read_int16(buffer + offset);
  offset += 2;
  result.motor_direction = read_int16(buffer + offset);
  offset += 2;
  result.incorrect_direction_threshold = read_int16(buffer + offset);
  offset += 2;
  result.max_pwm_change = read_int16(buffer + offset);
  offset += 2;
  result.max_angle_change = read_int16(buffer + offset);
  offset += 2;
  result.min_emf_voltage = read_int16(buffer + offset);
  offset += 2;
  result.hall_angle_ki = read_int16(buffer + offset);
  offset += 2;
  result.lead_angle_control_ki = read_int16(buffer + offset);
  offset += 2;
  result.torque_control_ki = read_int16(buffer + offset);
  offset += 2;
  result.battery_power_control_ki = read_int16(buffer + offset);
  offset += 2;
  result.speed_control_ki = read_int16(buffer + offset);
  offset += 2;
  result.probing_angular_speed = read_int16(buffer + offset);
  offset += 2;
  result.max_pwm_difference = read_int16(buffer + offset);
  offset += 2;
  result.emf_angle_error_variance_threshold = read_int16(buffer + offset);
  offset += 2;
  result.min_emf_for_motor_constant = read_int16(buffer + offset);
  offset += 2;
  result.max_resistive_power = read_int16(buffer + offset);
  offset += 2;
  result.resistive_power_ki = read_int16(buffer + offset);
  offset += 2;
  result.max_angular_speed = read_int16(buffer + offset);
  offset += 2;
  result.max_power_draw = read_int16(buffer + offset);
  offset += 2;
  result.power_draw_ki = read_int16(buffer + offset);
  offset += 2;
  result.max_pwm = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_torque_k_prediction = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_torque_ki = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_torque_kp = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_torque_kd = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_power_k_prediction = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_power_ki = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_power_kp = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_power_kd = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_speed_k_prediction = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_speed_ki = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_speed_kp = read_int16(buffer + offset);
  offset += 2;
  result.seek_via_speed_kd = read_int16(buffer + offset);
  offset += 2;
  result.phase_resistance = read_int16(buffer + offset);
  offset += 2;
  result.phase_inductance = read_int16(buffer + offset);
  offset += 2;
  return true;
}

static inline bool serialise(UnitTestOutput const& value, uint8_t * buffer) {
  size_t offset = 0;
  
  // Write message code
  write_uint16(buffer + offset, UnitTestOutput::message_code);
  offset += 2;
  
  for (size_t i = 0; i < 256; ++i) {
    write_uint8(buffer + offset, value.data[i]);
    offset += 1;
  }
  return true;
}

static inline bool deserialise(UnitTestOutput & result, uint8_t const* buffer, size_t length) {
  // Validate length
  if (length != UnitTestOutput::message_size) {
    return false;
  }
  
  // Validate message code
  uint16_t code = read_uint16(buffer);
  if (code != UnitTestOutput::message_code) {
    return false;
  }
  
  size_t offset = 2;
  
  for (size_t i = 0; i < 256; ++i) {
    result.data[i] = read_uint8(buffer + offset);
    offset += 1;
  }
  return true;
}

