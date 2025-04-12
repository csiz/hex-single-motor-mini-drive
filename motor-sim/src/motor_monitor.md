---
title: Motor monitor
---
<main class="hero">

Motor Commands
--------------

<div>${connect_buttons}</div>
<div>${motor_controller_status}</div>
<div>${data_stream_buttons}</div>
<div>${command_buttons}</div>
<div>
  ${command_value_slider}
  ${command_timeout_slider}
</div>

Motor Driving Data
------------------
<div class="card tight">
<p>Controls for the plotting time window:</p>
  ${time_period_input}
  ${time_offset_input}
  ${plot_options_input}
</div>
<div class="card tight">${plot_electric_position}</div>
<div class="card tight">${plot_speed}</div>
<div class="card tight">${plot_measured_current}</div>
<div class="card tight">${plot_dq0_currents}</div>
<div class="card tight">${plot_dq0_voltages}</div>
<div class="card tight">${plot_inferred_voltage}</div>
<div class="card tight">${plot_pwm_settings}</div>


Position Calibration Procedures
-------------------------------

<div class="card tight">
  <div>${position_calibration_buttons}</div>
  <div>Number of calibration data sets: ${position_calibration_results.length}</div>
</div>
<div class="card tight">
  <h3>Position Calibration Results</h3>
  <div>${position_calibration_result_to_display_input}</div>
  <div>${position_calibration_pos_plot}</div>
  <div>${position_calibration_pos_speed_plot}</div>
  <div>${position_calibration_neg_plot}</div>
  <div>${position_calibration_neg_speed_plot}</div>
  <div>${position_calibration_table}</div>
  <div>${position_calibration_hall_details_table}</div>
  <div>${position_calibration_hysterisis_table}</div>
</div>


Current Calibration Procedures
------------------------------

<div class="card tight">
  <div>${current_calibration_buttons}</div>
  <div>Number of calibration data sets: ${current_calibration_results.length}</div>
</div>
<div class="card tight">
  <h3>Current Calibration Results</h3>
  <div>${current_calibration_table}</div>
  <div>${current_calibration_result_to_display_input}</div>
  <div>${current_calibration_plot}</div>
  <div>${current_calibration_interpolate_plot}</div>
</div>
<div class="card tight">  
  <h3>Current Calibration Statistics</h3>
  <div>${current_calibration_positive_mean_plot}</div>
  <div>${current_calibration_negative_mean_plot}</div>
</div>


</main>

```js

const time_conversion = 1/23400 * 1000;

const adc_voltage_reference = 3.3;
const motor_shunt_resistance = 0.010;
const amplifier_gain = 20.0;
const adc_max_value = 0xFFF;
const current_conversion = adc_voltage_reference / (adc_max_value * motor_shunt_resistance * amplifier_gain);

const max_measurable_current = adc_max_value / 2 * current_conversion;

const drive_resistance = 2.0; // 2.0 Ohm measured with voltmeter between 1 phase and the other 2 in parallel.
const drive_voltage = 10.0; // 10.0 V // TODO: get it from the chip

const phase_inductance = 0.000_145; // 290 uH measured with LCR meter across phase pairs.
const phase_resistance = 2.0; // 2.0 Ohm

const π = Math.PI;

const std_95_z_score = 1.959964; // 95% confidence interval for normal distribution
const std_99_z_score = 2.575829; // 99% confidence interval for normal distribution
```

```js

let motor_controller = Mutable();

let motor_controller_status = Mutable(html`<span>Not connected.</span>`);

function update_motor_controller_status(status){
  motor_controller_status.value = status;
}


async function open_port_and_read(){
  if (motor_controller.value) {
    console.info("Forgetting previous port");
    try {
      await motor_controller.value.com_port.forget();
    } catch (error) {
      // Ignore network errors when forgetting, likely due to previous disconnect.
      if (error.name != "NetworkError") throw error;
    }
  }
  try {
    motor_controller.value = await motor.connect_usb_motor_controller();

    update_motor_controller_status(html`<span>Connected, waiting for data.</span>`);

    for await (const data_received of motor_controller.value.reading_loop()) {
      update_motor_controller_status(html`<span>Connected; received: ${data_received} bytes.</span>`);
    }

  } catch (error) {
    motor_controller.value = null;

    if (error.name === "SecurityError") {
      update_motor_controller_status(html`<span style="color: red">Permission for port dialog denied.</span>`);
      return;
    }

    if (error.message === "EOF") {
      update_motor_controller_status(html`<span style="color: red">End of connection.</span>`);
      return;
    }
    // Check for NetworkError due to disconnect.
    if (error.name === "NetworkError") {
      update_motor_controller_status(html`<span style="color: red">Connection lost.</span>`);
      return;
    }

    if (error.name === "NotFoundError") {
      update_motor_controller_status(html`<span style="color: red">No device found or nothing selected.</span>`);
      return;
    }

    update_motor_controller_status(html`<span style="color: red">Connection lost; unknown error: ${error}</span>`);
    throw error;
  }
}

invalidation.then(async function(){
  if (motor_controller.value) {
    console.info("Invalidating motor controller; forgetting port.");
    await motor_controller.value.com_port.forget();
    motor_controller.value = null;
    motor_controller_status.value = html`<span style="color: red">Invalidated!</span>`;
  }
});



const connect_buttons = Inputs.button(
  [
    ["Connect", open_port_and_read],
  ],
  {label: "Connect to COM"},
);

open_port_and_read();
```

```js
const command_value_slider = Inputs.range([0, 1], {value: 0.2, step: 0.05, label: "Command value:"});

const command_value_fraction = Generators.input(command_value_slider);

const command_timeout_slider = Inputs.range([0, motor.MAX_TIMEOUT*time_conversion], {value: 2000, step: 100, label: "Command timeout (ms):"});

const command_timeout_millis = Generators.input(command_timeout_slider);
```

```js
// Data stream output
// ------------------

let raw_readout_data = Mutable();

function update_raw_readout_data(new_data){
  raw_readout_data.value = new_data;
}

```

```js

// Control functions
// -----------------

const max_data_points = motor.HISTORY_SIZE;

const command_timeout = Math.floor(command_timeout_millis / time_conversion);
const command_value = Math.floor(command_value_fraction * motor.PWM_BASE);

async function command_and_stream(command, options = {}){
  if (!motor_controller) return;
    
  try {
    await motor_controller.send_command({command, command_timeout, command_value, ...options});

    // Start reading the data stream.
    for await (const data_snapshot of motor_controller.stream_readouts(options)) {
      update_raw_readout_data(data_snapshot.length > max_data_points ? data_snapshot.slice(-max_data_points) : data_snapshot);
    }
  } catch (error) {
    update_motor_controller_status(html`<span style="color: red">Error streaming data: ${error}</span>`);
    throw error;
  }
}

async function command(command){
  if (!motor_controller) return;
  
  await motor_controller.send_command({command, command_timeout, command_value});
}


const data_stream_buttons = Inputs.button(
  [
    ["ADC snapshot", async function(){
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE});
    }],
    ["Freewheel Snapshot", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE});
    }],
    ["ADC stream", async function(){
      await command_and_stream(motor.GET_READOUTS);
    }],
    ["Test all permutations", async function(){
      await command_and_stream(motor.SET_STATE_TEST_ALL_PERMUTATIONS);
    }],
    ["Test ground short", async function(){
      await command_and_stream(motor.SET_STATE_TEST_GROUND_SHORT);
    }],
    ["Test positive short", async function(){
      await command_and_stream(motor.SET_STATE_TEST_POSITIVE_SHORT);
    }],
    ["Test U directions", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_DIRECTIONS);
    }],
    ["Test U increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_INCREASING);
    }],
    ["Test U decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_U_DECREASING);
    }],
    ["Test V increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_V_INCREASING);
    }],
    ["Test V decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_V_DECREASING);
    }],
    ["Test W increasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_W_INCREASING);
    }],
    ["Test W decreasing", async function(){
      await command_and_stream(motor.SET_STATE_TEST_W_DECREASING);
    }],
  ],
  {label: "Read data"},
);

const command_buttons = Inputs.button(
  [
    ["Stop", async function(){
      await command(motor.SET_STATE_OFF);
    }],
    ["Drive+", async function(){
      await command(motor.SET_STATE_DRIVE_POS);
    }],
    ["Drive-", async function(){
      await command(motor.SET_STATE_DRIVE_NEG);
    }],
    ["Freewheel", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
    }],
    ["Hold U positive", async function(){
      await command(motor.SET_STATE_HOLD_U_POSITIVE);
    }],
    ["Hold V positive", async function(){
      await command(motor.SET_STATE_HOLD_V_POSITIVE);
    }],
    ["Hold W positive", async function(){
      await command(motor.SET_STATE_HOLD_W_POSITIVE);
    }],
    ["Hold U negative", async function(){
      await command(motor.SET_STATE_HOLD_U_NEGATIVE);
    }],
    ["Hold V negative", async function(){
      await command(motor.SET_STATE_HOLD_V_NEGATIVE);
    }],
    ["Hold W negative", async function(){
      await command(motor.SET_STATE_HOLD_W_NEGATIVE);
    }],
  ],
  {label: "Commands"},
);

```


```js

// Load Calibration
// ----------------


const current_calibration_default = {
  u_positive: 1.0,
  u_negative: 1.0,
  v_positive: 1.0,
  v_negative: 1.0,
  w_positive: 1.0,
  w_negative: 1.0,
};

function load_current_calibration_factors(){
  return get_stored_or_default("current_calibration", current_calibration_default);
}

// Load or make default calibration
const saved_current_calibration_factors = load_current_calibration_factors();

let current_calibration_factors = Mutable(saved_current_calibration_factors);

function update_current_calibration_factors(new_calibration_factors){
  current_calibration_factors.value = new_calibration_factors;
}

function reload_current_calibration_factors(){
  update_current_calibration_factors(load_current_calibration_factors());
}

function save_current_calibration_factors(){
  localStorage.setItem("current_calibration", JSON.stringify(current_calibration_factors.value));
}

function reset_current_calibration_factors(){
  localStorage.removeItem("current_calibration");
  update_current_calibration_factors(current_calibration_default);
}
```


```js

// Calculate Data
// --------------

function matrix_multiply(m, v){
  // Return the matrix-vector product of a 3x3 matrix and a 3-vector.
  return m.map((row) => d3.sum(row.map((x, i) => x * v[i])));
}

const power_invariant_clarke_matrix = [
  [Math.sqrt(2/3), -0.5 * Math.sqrt(2/3), -0.5 * Math.sqrt(2/3)],
  [0, Math.sqrt(2)/2, -Math.sqrt(2)/2],
  [1/Math.sqrt(3), 1/Math.sqrt(3), 1/Math.sqrt(3)],
];

const power_invariant_simplified_clarke_matrix = [
  [Math.sqrt(2/3), -0.5 * Math.sqrt(2/3), -0.5 * Math.sqrt(2/3)],
  [0, Math.sqrt(2)/2, -Math.sqrt(2)/2],
];


function normalize_degrees(a){
  return (a % 360 + 540) % 360 - 180;
}

function shortest_distance_degrees(a, b){
  return normalize_degrees(b - a);
}


function interpolate_degrees(a, b, fraction){
  const diff = b - a;
  return normalize_degrees(a + diff * fraction);
}

function interpolate_linear(a, b, fraction){
  return a + (b - a) * fraction;
}


function circular_stats_degrees(values){
  const mean_point = [
    d3.mean(values, (d) => Math.cos(d * Math.PI / 180.0)),
    d3.mean(values, (d) => Math.sin(d * Math.PI / 180.0)),
  ]
  const circular_mean = Math.atan2(mean_point[1], mean_point[0]) * 180 / Math.PI;

  const circular_σ = Math.sqrt(
    d3.mean(values, (d) => {
      const diff = shortest_distance_degrees(circular_mean, d);
      return diff * diff;
    })
  );

  return {
    circular_mean,
    circular_σ,
  };
}

function hall_sector({hall_u, hall_v, hall_w}){
  const hall_state = (hall_u ? 0b001 : 0) | (hall_v ? 0b010 : 0) | (hall_w ? 0b100 : 0);
  switch(hall_state){
    case 0b001: return 0;
    case 0b011: return 1;
    case 0b010: return 2;
    case 0b110: return 3;
    case 0b100: return 4;
    case 0b101: return 5;
    default: return null;
  }
}

const sector_center_degrees = [0, 60, 120, 180, 240, 300].map(normalize_degrees);
const sector_center_σ = [30, 30, 30, 30, 30, 30];
const hall_hysterisis = 10;
const transition_σ = 15;
const sector_transition_degrees = [
  [- 30 + hall_hysterisis / 2, + 30 - hall_hysterisis / 2],
  [+ 30 + hall_hysterisis / 2, + 90 - hall_hysterisis / 2],
  [+ 90 + hall_hysterisis / 2, +150 - hall_hysterisis / 2],
  [+150 + hall_hysterisis / 2, -150 - hall_hysterisis / 2],
  [-150 + hall_hysterisis / 2, - 90 - hall_hysterisis / 2],
  [- 90 + hall_hysterisis / 2, - 30 - hall_hysterisis / 2],
];
const sector_transition_σ = [
  [transition_σ, transition_σ],
  [transition_σ, transition_σ],
  [transition_σ, transition_σ],
  [transition_σ, transition_σ],
  [transition_σ, transition_σ],
  [transition_σ, transition_σ],
];
const accel_σ = 360.0 / 5.0 / 50.0; // acceleration distribution up to (360 degrees per 5ms) per 50ms.
const initial_angular_speed_σ = 0.05 * 360;

function shortest_distance_mod_6(a, b){
  const diff = (b + 12 - a) % 6;
  return diff > 3 ? diff - 6 : diff;
}

function product_of_normals({mean_a, σ_a, mean_b, σ_b}){
  const mean = (mean_a * σ_b * σ_b + mean_b * σ_a * σ_a) / (σ_a * σ_a + σ_b * σ_b);
  const σ = Math.sqrt((σ_a * σ_a * σ_b * σ_b) / (σ_a * σ_a + σ_b * σ_b));
  return {mean, σ};
}

function product_normals_2({mean_a, variance_a, mean_b, variance_b}){
  const mean = (mean_a * variance_b + mean_b * variance_a) / (variance_a + variance_b);
  const variance = (variance_a * variance_b) / (variance_a + variance_b);
  return {mean, variance};
}

function add_σ(...σ_values){
  return Math.sqrt(σ_values.reduce((sum, σ) => sum + σ * σ, 0));
}


function compute_position_from_hall(data){
  if (!Array.isArray(data) || data.length < 1) return data;

  const first_sector = hall_sector(data[0]);
  
  let last_event = {
    time: 0,
    sector: first_sector,
    angle: sector_center_degrees[first_sector],
    angle_σ: sector_center_σ[first_sector],
    angular_speed: 0,
    angular_speed_σ: initial_angular_speed_σ,
  };

  let result = [{...data[0], ...last_event}];

  for (let i = 1; i < data.length; i++){
    const d = data[i];

    const {time} = d;

    const dt = time - last_event.time;


    const sector = hall_sector(d);

    // If we switched sectors, we should have an accurate position estimate.
    if (sector != null && sector != last_event.sector){
      const direction = Math.sign(shortest_distance_mod_6(last_event.sector, sector));

      const trigger_angle = sector_transition_degrees[sector][direction >= 0 ? 0 : 1];
      const trigger_angle_σ = sector_transition_σ[sector][direction >= 0 ? 0 : 1];

      const distance_to_trigger = shortest_distance_degrees(last_event.angle, trigger_angle);

      const estimated_distance = last_event.angular_speed * dt;
      const estimated_distance_error = distance_to_trigger - estimated_distance;
      const estimated_speed_error = estimated_distance_error / dt;

      const estimated_distance_σ = add_σ(last_event.angle_σ, last_event.angular_speed_σ * dt, accel_σ * dt * dt / 2.0);
      const estimated_speed_error_σ = add_σ(estimated_distance_σ / dt, trigger_angle_σ / dt);
      
      const {mean: distance_adjustment, σ: angle_σ} = product_of_normals({
        mean_a: 0.0,
        σ_a: estimated_distance_σ,
        mean_b: estimated_distance_error,
        σ_b: trigger_angle_σ,
      });

      const kalman_angle = normalize_degrees(last_event.angle + estimated_distance + distance_adjustment);

      // Ensure we dragged the angle within the 95% confidence interval of the trigger angle. If we're too slow
      // to update the angle we cross 180 degrees and our math switches sign. To avoid that we need to keep the 
      // angle near the trigger. The formula below handles the lower bound of the trigger angle; the upper bound
      // is handled by capping the speed above.
      const angle = kalman_angle + direction * Math.max(direction * normalize_degrees(trigger_angle - direction * std_95_z_score * trigger_angle_σ - kalman_angle), 0);

      const {mean: speed_adjustment, σ: angular_speed_σ} = product_of_normals({
        mean_a: 0.0,
        σ_a: last_event.angular_speed_σ + 0.5 * accel_σ * dt,
        mean_b: estimated_speed_error,
        σ_b: estimated_speed_error_σ,
      });

      const angular_speed = last_event.angular_speed + speed_adjustment;

      last_event = {
        time,
        sector,
        angle,
        angle_σ,
        angular_speed,
        angular_speed_σ,
      };
        

      result.push({...d, sector, angle, angle_σ, angular_speed, angular_speed_σ});

    // If we didn't switch sectors, then we need to carry on with our previous estimate, but adjusted...
    } else {
      const positive_direction = last_event.angular_speed >= 0;
      const direction = positive_direction ? +1 : -1;

      const estimated_distance = last_event.angular_speed * dt;
      const estimated_distance_σ = add_σ(last_event.angle_σ, last_event.angular_speed_σ * dt, accel_σ * dt * dt / 2.0);

      const next_sector = positive_direction ? (last_event.sector + 1) % 6 : (last_event.sector - 1 + 6) % 6;
      const next_transition_angle = sector_transition_degrees[next_sector][positive_direction ? 0 : 1];
      const next_transition_angle_σ = sector_transition_σ[next_sector][positive_direction ? 0 : 1];
      // Cap the position in the 95% confidence interval before the next transition. We cross
      // this threshold when distance_to_next_transition is negative. Also the adjustement
      // depends on the direction of travel.
      const distance_to_next_transition = direction * shortest_distance_degrees(last_event.angle, next_transition_angle) + std_95_z_score * next_transition_angle_σ;
      
      const distance_overshoot = Math.max(direction * estimated_distance - distance_to_next_transition, 0);
      const distance_adjustment = direction * Math.min(distance_to_next_transition - direction * estimated_distance, 0);

      const estimated_angle = normalize_degrees(last_event.angle + estimated_distance + distance_adjustment);

      const p_stopped_in_current_sector = distance_overshoot < 90 ? 0.0 : Math.min(1.0, (distance_overshoot - 90) / 45);

      const current_sector_angle = sector_center_degrees[last_event.sector];
      const current_sector_angle_σ = sector_center_σ[last_event.sector];

      const angle = interpolate_degrees(estimated_angle, current_sector_angle, p_stopped_in_current_sector);
      const angle_σ = interpolate_linear(estimated_distance_σ, current_sector_angle_σ, p_stopped_in_current_sector);

      const calculated_spin = shortest_distance_degrees(last_event.angle, angle) / dt;
      const calculated_spin_σ = (angle_σ + last_event.angle_σ) / dt;

      const {mean: angular_speed, σ: angular_speed_σ} = product_of_normals({
        mean_a: calculated_spin,
        σ_a: calculated_spin_σ,
        mean_b: last_event.angular_speed,
        σ_b: last_event.angular_speed_σ + accel_σ * dt,
      });

      if (distance_overshoot > 90 + 45){
        last_event = {
          time,
          sector: next_sector,
          angle: next_transition_angle,
          angle_σ: next_transition_angle_σ,
          angular_speed: 0.0,
          angular_speed_σ: initial_angular_speed_σ,
        };
      }

      result.push({...d, sector, angle, angle_σ, angular_speed, angular_speed_σ});
    }
  }

  return result;
}


function calculate_data_stats(raw_readout_data){
  
  
  const ref_readout_mean = d3.mean(raw_readout_data, (d) => d.ref_readout);
  const start_readout_number = raw_readout_data[0].readout_number;

  const data_with_annotations = raw_readout_data.map((d) => {
    const readout_number = (motor.READOUT_BASE + d.readout_number - start_readout_number) % motor.READOUT_BASE;
    const time = readout_number * time_conversion;
  
    const u_readout = -current_conversion * (d.u_readout - d.ref_readout);
    const v_readout = +current_conversion * (d.v_readout - d.ref_readout);
    const w_readout = -current_conversion * (d.w_readout - d.ref_readout);
    const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);
  
    const calibrated_u = u_readout * (u_readout >= 0 ? 
      current_calibration_factors.u_positive :
      current_calibration_factors.u_negative);

    const calibrated_v = v_readout * (v_readout >= 0 ?
      current_calibration_factors.v_positive :
      current_calibration_factors.v_negative);

    const calibrated_w = w_readout * (w_readout >= 0 ?
      current_calibration_factors.w_positive :
      current_calibration_factors.w_negative);

    const sum = calibrated_u + calibrated_v + calibrated_w;

    const u = calibrated_u - sum / 3.0;
    const v = calibrated_v - sum / 3.0;
    const w = calibrated_w - sum / 3.0;

    const [current_alpha, current_beta] = matrix_multiply(power_invariant_simplified_clarke_matrix, [u, v, w]);

    const current_angle = Math.atan2(current_beta, current_alpha);
    const current_magnitude = Math.sqrt(current_alpha * current_alpha + current_beta * current_beta);

    // Approximate the angle for the 6 sectors of the hall sensor.
    const ε = 0.1;
    const hall_u_as_angle = d.hall_u ? d.hall_v ? +π/3 -ε : d.hall_w ? -π/3 +ε : 0.0 : null;
    const hall_v_as_angle = d.hall_v ? d.hall_u ? +π/3 +ε : d.hall_w ? +π -ε : +2*π/3 : null;
    const hall_w_as_angle = d.hall_w ? d.hall_v ? -π +ε : d.hall_u ? -π/3 -ε : -2*π/3 : null;


    return {
      ...d, 
      motor_angle: normalize_degrees(d.motor_angle * 360 / 256),
      motor_speed_σ: Math.sqrt(d.motor_speed_variance),
      u_readout, v_readout, w_readout, 
      u, v, w, 
      ref_diff, time, sum, 
      current_alpha, current_beta, 
      current_angle: current_angle * 180 / Math.PI, 
      current_magnitude, 
      hall_u_as_angle: hall_u_as_angle === null ? null : hall_u_as_angle * 180 / Math.PI, 
      hall_v_as_angle: hall_v_as_angle === null ? null : hall_v_as_angle * 180 / Math.PI, 
      hall_w_as_angle: hall_w_as_angle === null ? null : hall_w_as_angle * 180 / Math.PI,
    };
  });

  const derivative_start_index = 8;
  const derivative_end_index = raw_readout_data.length - 2;

  function derivative_3_points(index, df, data, circular = false){
    if (index < derivative_start_index || index > derivative_end_index) return null;
    const left = data[index - 1];
    const mid = data[index];
    const right = data[index + 1];
    const left_derivative = df(left, mid) / (mid.time - left.time);
    const right_derivative = df(mid, right) / (right.time - mid.time);
    return (left_derivative + right_derivative) / 2.0;
  }

  function inductor_voltage_3_points(index, df, data){
    const derivative = derivative_3_points(index, df, data);
    return derivative == null ? null : derivative * 1000 * phase_inductance; // V = L*dI/dt + R*I
  }

  const data_with_derivatives = data_with_annotations.map((d, i, data) => {
    const current_angular_speed = derivative_3_points(i, (a, b) => normalize_degrees(b.current_angle - a.current_angle), data);
    const u_L_voltage = inductor_voltage_3_points(i, (a, b) => b.u - a.u, data);
    const v_L_voltage = inductor_voltage_3_points(i, (a, b) => b.v - a.v, data);
    const w_L_voltage = inductor_voltage_3_points(i, (a, b) => b.w - a.w, data);
    
    const u_voltage = u_L_voltage === null ? null : u_L_voltage + phase_resistance * d.u;
    const v_voltage = v_L_voltage === null ? null : v_L_voltage + phase_resistance * d.v;
    const w_voltage = w_L_voltage === null ? null : w_L_voltage + phase_resistance * d.w;

    const any_null = current_angular_speed === null || u_L_voltage === null || v_L_voltage === null || w_L_voltage === null;

    const [voltage_alpha, voltage_beta] = any_null ? [null, null] : matrix_multiply(power_invariant_clarke_matrix, [u_voltage, v_voltage, w_voltage]);
    const voltage_angle = any_null ? null : Math.atan2(voltage_beta, voltage_alpha);
    
    const voltage_magnitude = any_null ? null : Math.sqrt(voltage_alpha * voltage_alpha + voltage_beta * voltage_beta);

    const angle_if_breaking = any_null ? null : ((voltage_angle + (current_angular_speed > 0 ? +Math.PI / 2.0 : -Math.PI / 2.0)) + 3 * Math.PI) % (2 * Math.PI) - Math.PI;

    return {
      ...d,
      current_angular_speed,
      u_voltage,
      v_voltage,
      w_voltage,
      u_L_voltage,
      v_L_voltage,
      w_L_voltage,
      voltage_alpha,
      voltage_beta,
      voltage_angle: voltage_angle === null ? null : voltage_angle * 180 / Math.PI,
      voltage_magnitude,
      angle_if_breaking: angle_if_breaking === null ? null : angle_if_breaking * 180 / Math.PI,
    };
  });

  const data_with_positions = compute_position_from_hall(data_with_derivatives);

  const data = data_with_positions;

  return {data, ref_readout_mean, start_readout_number};
}
```


```js
const {data, ref_readout_mean} = calculate_data_stats(raw_readout_data);

```



```js

// Plotting
// --------


const colors = {
  u: "rgb(117, 112, 179)",
  v: "rgb(217, 95, 2)",
  w: "rgb(231, 41, 138)",
  ref_diff: "rgb(102, 102, 102)",
  sum: "black",
  angle: "black",
  motor_angle: "rgb(39, 163, 185)",
  current_magnitude: "rgb(197, 152, 67)",
  current_angle: "rgb(102, 166, 30)",
  voltage_angle: d3.color("rgb(102, 166, 30)").darker(1),
  angle_if_breaking: d3.color("rgb(102, 166, 30)").darker(2),
  angular_speed: "black",
  current_angular_speed: "rgb(27, 158, 119)",
  current_alpha: "rgb(199, 0, 57)",
  current_beta: "rgb(26, 82, 118)",
};


```

```js

const history_duration = Math.ceil(motor.HISTORY_SIZE * time_conversion);

const time_period_input = Inputs.range([1, 50], {
  value: history_duration,
  transform: Math.log,
  step: 0.5,
  label: "Time window Duration (ms):",
});
const time_period = Generators.input(time_period_input);

const time_offset_input = Inputs.range([-1.0, 1.0], {
  value: 0, 
  step: 0.01,
  label: "Time window Rewind (ms):",
});
d3.select(time_offset_input).select("div").style("width", "640px");
d3.select(time_offset_input).select("input[type=range]").style("width", "100em");

const time_offset = Generators.input(time_offset_input);

const plot_options_input = Inputs.checkbox(
  ["Connected lines"],
  {
    value: ["Connected lines"],
    label: "Plot options:",
  },
);

const plot_options = Generators.input(plot_options_input);
```

```js
const data_duration = data[data.length - 1].time;

const time_end = Math.max(+time_period * 0.5, Math.min(data_duration * (1.0 - time_offset), data_duration + time_period * 0.5));
const time_domain = [time_end - time_period, time_end];

const selected_data = data.filter((d) => d.time >= time_domain[0] && d.time <= time_domain[1]);

const plot_electric_position = plot_multiline({
  data: selected_data,
  store_id: "plot_electric_position",
  selection: null,
  subtitle: "Electric position",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_options: {domain: time_domain},
  y_options: {domain: [-180, 180]},
  x: "time",
  y: "current_angle",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channel_label: "Angle Source",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle,
      y1: (d) => d.angle - std_95_z_score * d.angle_σ,
      y2: (d) => d.angle + std_95_z_score * d.angle_σ,
    },
    {y: "motor_angle", label: "Motor Angle", color: colors.motor_angle},
    {y: "current_angle", label: "Current (Park) Angle", color: colors.current_angle},
    {y: "voltage_angle", label: "Voltage (Park) Angle", color: colors.voltage_angle},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 90, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_speed = plot_multiline({
  data: selected_data,
  store_id: "plot_speed",
  selection: null,
  subtitle: "Rotor Speed",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "current_angular_speed",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channel_label: "Speed Source",
  channels: [
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      y1: (d) => d.angular_speed - std_95_z_score * d.angular_speed_σ,
      y2: (d) => d.angular_speed + std_95_z_score * d.angular_speed_σ,
    },
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_measured_current = plot_multiline({
  data: selected_data,
  store_id: "plot_measured_current",
  selection: null,
  subtitle: "Measured Current",
  description: "Measured current values for each phase.",
  width: 1200, height: 400,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "u",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channel_label: "Phase",
  channels: [
    {y: "u", label: "Current U", color: colors.u},
    {y: "v", label: "Current V", color: colors.v},
    {y: "w", label: "Current W", color: colors.w},
    {y: "sum", label: "Sum", color: colors.sum},
    {y: "ref_diff", label: "Ref Diff", color: colors.ref_diff},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_dq0_currents = plot_multiline({
  data: selected_data,
  store_id: "plot_dq0_currents",
  selection: null,
  subtitle: "DQ0 Currents",
  description: "DQ0 currents after Clarke and Park transforming the measured currents.",
  width: 1200, height: 400,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "current_alpha",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channel_label: "Phase",
  channels: [
    {y: "current_alpha", label: "Current Alpha", color: colors.current_alpha},
    {y: "current_beta", label: "Current Beta", color: colors.current_beta},
    {y: "current_magnitude", label: "Current (Park) Magnitude", color: colors.current_magnitude},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_dq0_voltages = plot_multiline({
  data: selected_data,
  store_id: "plot_dq0_voltages",
  selection: null,
  subtitle: "DQ0 Voltages",
  description: "DQ0 voltages after Clarke and Park transforming the inferred voltages.",
  width: 1200, height: 400,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "voltage_alpha",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channel_label: "Phase",
  channels: [
    {y: "voltage_alpha", label: "Voltage Alpha", color: colors.current_alpha},
    {y: "voltage_beta", label: "Voltage Beta", color: colors.current_beta},
    {y: "voltage_magnitude", label: "Voltage (Park) Magnitude", color: colors.current_magnitude},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_inferred_voltage = plot_multiline({
  data: selected_data,
  store_id: "plot_inferred_voltage",
  selection: null,
  subtitle: "Inferred Voltage",
  description: html`Inferred voltage values for each phase: ${tex`V = IR + L(dI/dt)`}.`,
  width: 1200, height: 300,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "u_voltage",
  x_label: "Time (ms)",
  y_label: "Voltage (V)",
  channel_label: "Phase",
  channels: [
    {y: "u_voltage", label: "Voltage U", color: colors.u},
    {y: "v_voltage", label: "Voltage V", color: colors.v},
    {y: "w_voltage", label: "Voltage W", color: colors.w},
    {y: "u_L_voltage", label: "Inductor Voltage U", color: d3.color(colors.u).brighter(1)},
    {y: "v_L_voltage", label: "Inductor Voltage V", color: d3.color(colors.v).brighter(1)},
    {y: "w_L_voltage", label: "Inductor Voltage W", color: d3.color(colors.w).brighter(1)},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_pwm_settings = plot_multiline({
  data: selected_data,
  store_id: "plot_pwm_settings",
  selection: null,
  subtitle: "PWM Settings",
  description: "The PWM value currently set for each phase.",
  width: 1200, height: 300,
  x_options: {domain: time_domain},
  y_options: {domain: [0, motor.PWM_BASE]},
  x: "time",
  y: "u_pwm",
  x_label: "Time (ms)",
  y_label: "PWM",
  channel_label: "Phase",
  channels: [
    {y: "u_pwm", label: "PWM U", color: colors.u},
    {y: "v_pwm", label: "PWM V", color: colors.v},
    {y: "w_pwm", label: "PWM W", color: colors.w},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 128, stroke: 'black', strokeWidth : 1}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

```

```js
// Position Calibration
// --------------------

let position_calibration_results = Mutable();

function store_position_calibration_results(calibration_data){
  position_calibration_results.value = [...position_calibration_results.value ?? [], calibration_data];
}
```

```js
const position_calibration_buttons = Inputs.button(
  [
    ["Start Position Calibration", async function(){
      for (let i = 0; i < 10; i++){
        await run_position_calibration();
      }
    }],
  ],
  {label: "Collect position calibration data"},
);

d3.select(position_calibration_buttons).style("width", "100%");


async function run_position_calibration(){
  if (!motor_controller) return;

  console.info("Position calibration starting");
  
  const drive_time = 200;
  const drive_timeout = Math.floor((drive_time + 300) / time_conversion);

  const drive_strength = Math.floor(motor.PWM_BASE * 2 / 10);
  const drive_options = {command_timeout: drive_timeout, command_value: drive_strength};

  const test_options = {command_timeout: 0, command_value: 0};

  await motor_controller.send_command({command: motor.SET_STATE_DRIVE_POS, ...drive_options});  
  await wait(drive_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_GROUND_SHORT, ...test_options});

  const drive_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("Drive positive done");

  await motor_controller.send_command({command: motor.SET_STATE_DRIVE_NEG, ...drive_options});
  await wait(drive_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_GROUND_SHORT, ...test_options});
  
  const drive_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;
  
  
  console.info("Drive negative done");

  console.info("Position calibration done");

  if (drive_positive.length != motor.HISTORY_SIZE) {
    console.error("Drive positive data is not valid", drive_positive);
    return;
  }
  if (drive_negative.length != motor.HISTORY_SIZE) {
    console.error("Drive negative data is not valid", drive_negative);
    return;
  }

  const position_calibration_data = {
    drive_positive,
    drive_negative,
    concatenated: [
      ...drive_positive.map(d => ({...d, direction: "positive"})),
      ...drive_negative.map(d => ({...d, direction: "negative"})),
    ],
  };
  
  store_position_calibration_results(position_calibration_data);
}
```


```js
const position_calibration_result_to_display_input = Inputs.select(
  d3.range(0, position_calibration_results.length),
  {
    value: position_calibration_results.length - 1,
    label: "Select position calibration data set:",
  },
);
const position_calibration_result_to_display = Generators.input(position_calibration_result_to_display_input);
```


```js

const position_calibration_selected_result = position_calibration_results[position_calibration_result_to_display];
const position_calibration_detailed_result = {
  drive_positive: position_calibration_selected_result.drive_positive,
  drive_negative: position_calibration_selected_result.drive_negative,
};

const position_calibration_pos_plot = plot_multiline({
  data: position_calibration_detailed_result.drive_positive,
  store_id: "position_calibration_pos_plot",
  selection: null,
  subtitle: "Electric position | drive positive then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {domain: [-180, 180]},
  x: "time",
  y: "current_angle",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channel_label: "Angle Source",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle,
      y1: (d) => d.angle - std_95_z_score * d.angle_σ,
      y2: (d) => d.angle + std_95_z_score * d.angle_σ,
    },
    {y: "angle_if_breaking", label: "Angle If Breaking", color: colors.angle_if_breaking},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 90, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: "step",
});

function clipped_lower(f, data, y){
  const lower_bound = d3.min(data, (d) => d[y]);
  if (lower_bound == null) return f;
  return (d, i, data) => Math.max(f(d, i, data), lower_bound);
}
function clipped_upper(f, data, y){
  const upper_bound = d3.max(data, (d) => d[y]);
  if (upper_bound == null) return f;
  return (d, i, data) => Math.min(f(d, i, data), upper_bound);
}

const position_calibration_pos_speed_plot = plot_multiline({
  data: position_calibration_detailed_result.drive_positive,
  store_id: "position_calibration_pos_speed_plot",
  selection: null,
  subtitle: "Rotor Speed | drive positive then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {},
  x: "time",
  y: "current_angular_speed",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channel_label: "Speed Source",
  channels: [
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed, 
      y1: (d) => d.angular_speed - std_95_z_score * d.angular_speed_σ,
      y2: (d) => d.angular_speed + std_95_z_score * d.angular_speed_σ,
    },
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: "step",
});

const position_calibration_neg_plot = plot_multiline({
  data: position_calibration_detailed_result.drive_negative,
  store_id: "position_calibration_neg_plot",
  selection: null,
  subtitle: "Electric position | drive negative then break",
  description: "Angular position of the rotor with respect to the electric phases, 0 when magnetic N is aligned with phase U.",
  width: 1200, height: 150,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {domain: [-180, 180]},
  x: "time",
  y: "current_angle",
  x_label: "Time (ms)",
  y_label: "Electric position (degrees)",
  channel_label: "Angle Source",
  channels: [
    {
      y: "angle", label: "Angle", color: colors.angle, 
      y1: (d) => d.angle - std_95_z_score * d.angle_σ,
      y2: (d) => d.angle + std_95_z_score * d.angle_σ,
    },
    {y: "angle_if_breaking", label: "Angle If Breaking", color: d3.color(colors.current_angle).darker(2)},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 90, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: "step",
});

const position_calibration_neg_speed_plot = plot_multiline({
  data: position_calibration_detailed_result.drive_negative,
  store_id: "position_calibration_neg_speed_plot",
  selection: null,
  subtitle: "Rotor Speed | drive negative then break",
  description: "Angular speed of the rotor in degrees per millisecond.",
  width: 1200, height: 150,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {},
  x: "time",
  y: "current_angular_speed",
  x_label: "Time (ms)",
  y_label: "Angular Speed (degrees/ms)",
  channel_label: "Speed Source",
  channels: [
    {y: "current_angular_speed", label: "Current Speed", color: colors.current_angular_speed},
    {
      y: "angular_speed", label: "Angular Speed", color: colors.angular_speed,
      y1: (d) => d.angular_speed - std_95_z_score * d.angular_speed_σ,
      y2: (d) => d.angular_speed + std_95_z_score * d.angular_speed_σ,
    },
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ],
  curve: "step",
});
```

```js

// Store the inferred angle for every hall sensor switching event.
function extract_hall_switching_events(data){
  return data.flatMap((d, i) => {
    if (d.angle_if_breaking == null) return [];
    if (i >= data.length - 1) return [];
    const next = data[i + 1];
    const angle_if_breaking = interpolate_degrees(d.angle_if_breaking, next.angle_if_breaking, 0.5);

    const prev_sector = hall_sector(d);
    const next_sector = hall_sector(next);

    if (prev_sector == null || next_sector == null) return [];

    if (prev_sector == next_sector) return [];

    return [{time: d.time, angle_if_breaking, switching: `hall_sector_${prev_sector}_to_${next_sector}`}];

  });
}

const position_calibration_switching_events = position_calibration_results.flatMap((calibration_data) => {
  return extract_hall_switching_events(calibration_data.concatenated);
});

const position_calibration_switching_angles = position_calibration_switching_events.reduce((acc, d) => {
  acc[d.switching].push(d.angle_if_breaking);
  return acc;
}, {
  hall_sector_5_to_0: [],
  hall_sector_0_to_1: [],
  hall_sector_1_to_2: [],
  hall_sector_2_to_3: [],
  hall_sector_3_to_4: [],
  hall_sector_4_to_5: [],

  hall_sector_1_to_0: [], 
  hall_sector_2_to_1: [],
  hall_sector_3_to_2: [],
  hall_sector_4_to_3: [],
  hall_sector_5_to_4: [],
  hall_sector_0_to_5: [],
});

const position_calibration_statistics = Object.entries(position_calibration_switching_angles).map(([key, values]) => {
  const { circular_mean: mean, circular_σ: σ } = circular_stats_degrees(values);
  return {key, n: values.length, mean, σ};
});

const position_calibration_table = Inputs.table(position_calibration_statistics, {rows: 12+1});

```

```js

const position_calibration_hall_details = [
  ["drive_positive_hall_u", "hall_sector_4_to_5", "hall_sector_1_to_2"],
  ["drive_negative_hall_u", "hall_sector_5_to_4", "hall_sector_2_to_1"],
  ["drive_positive_hall_v", "hall_sector_0_to_1", "hall_sector_3_to_4"],
  ["drive_negative_hall_v", "hall_sector_1_to_0", "hall_sector_4_to_3"],
  ["drive_positive_hall_w", "hall_sector_2_to_3", "hall_sector_5_to_0"],
  ["drive_negative_hall_w", "hall_sector_3_to_2", "hall_sector_0_to_5"],
].map(([key, a_key, b_key]) => {
  const a_stats = position_calibration_statistics.find(d => d.key === a_key);
  const b_stats = position_calibration_statistics.find(d => d.key === b_key);

  const a = a_stats.mean;
  const b = b_stats.mean;

  const diff = (b - a + 360) % 360;
  const location = interpolate_degrees(a, b, 0.5);
  const err = diff - 180;
  return {key, a, b, diff, location, err};
});

const position_calibration_hall_details_table = Inputs.table(position_calibration_hall_details, {rows: 6+1});

const position_calibration_hysterisis = [
  ["hysterisis_hall_u", "drive_negative_hall_u", "drive_positive_hall_u"],
  ["hysterisis_hall_v", "drive_negative_hall_v", "drive_positive_hall_v"],
  ["hysterisis_hall_w", "drive_negative_hall_w", "drive_positive_hall_w"],
].map(([key, a_key, b_key]) => {
  const a_stats = position_calibration_hall_details.find(d => d.key === a_key);
  const b_stats = position_calibration_hall_details.find(d => d.key === b_key);
  const hysterisis = shortest_distance_degrees(a_stats.location, b_stats.location);
  const mid_location = interpolate_degrees(a_stats.location, b_stats.location, 0.5);

  return {key, hysterisis, mid_location};
});

const position_calibration_hysterisis_table = Inputs.table(position_calibration_hysterisis, {rows: 3+1});
```


```js
// Current calibration
// -------------------

let calculated_current_calibration_factors = Mutable(current_calibration_factors);

function update_current_calibration(new_calibration){
  // Save calibration factors if valid.
  if (new_calibration === null) {
    console.error("Calibration data is not valid", new_calibration);
    return;
  }
  if (Object.values(new_calibration).some((data) => data === null)) {
    console.error("Calibration data is not valid", new_calibration);
    return;
  }
  calculated_current_calibration_factors.value = {
    u_positive: new_calibration.u_positive.factor,
    u_negative: new_calibration.u_negative.factor,
    v_positive: new_calibration.v_positive.factor,
    v_negative: new_calibration.v_negative.factor,
    w_positive: new_calibration.w_positive.factor,
    w_negative: new_calibration.w_negative.factor,
  };
}
```


```js

const current_calibration_buttons = Inputs.button(
  [
    ["Start Current Calibration", async function(){
      await run_current_calibration();
    }],
    ["Save Current Factors", function(){
      update_current_calibration_factors(calculated_current_calibration_factors);
      save_current_calibration_factors();
    }],
    ["Reload Current Factors", function(){
      reload_current_calibration_factors();
    }],
    ["Reset Current Factors", function(){
      reset_current_calibration_factors();
    }],
  ],
  {label: "Collect current calibration data"},
);

d3.select(current_calibration_buttons).style("width", "100%");
```

```js
let current_calibration_results = Mutable();

function store_current_calibration_results(calibration_data){
  current_calibration_results.value = [...current_calibration_results.value ?? [], calibration_data];
}
```


```js
const short_duration = motor.HISTORY_SIZE / 12 * time_conversion;

const current_calibration_zones = [
  {pwm: 0.1, settle_start: short_duration * 1.4, settle_end: short_duration * 1.95},
  {pwm: 0.2, settle_start: short_duration * 2.4, settle_end: short_duration * 2.95},
  {pwm: 0.3, settle_start: short_duration * 3.4, settle_end: short_duration * 3.95},
  {pwm: 0.4, settle_start: short_duration * 4.4, settle_end: short_duration * 4.95},
  {pwm: 0.5, settle_start: short_duration * 5.4, settle_end: short_duration * 5.95},
  {pwm: 0.6, settle_start: short_duration * 6.4, settle_end: short_duration * 6.95},
  {pwm: 0.7, settle_start: short_duration * 7.4, settle_end: short_duration * 7.95},
  {pwm: 0.8, settle_start: short_duration * 8.4, settle_end: short_duration * 8.95},
  {pwm: 0.9, settle_start: short_duration * 9.4, settle_end: short_duration * 9.95},
];


const calibration_reference = drive_voltage / drive_resistance;
const current_calibration_points = 32;
const current_calibration_reading_points = even_spacing(calibration_reference, current_calibration_points / 2 + 1);


async function run_current_calibration(){
  if (!motor_controller) return;
  
  const settle_time = 200;
  const settle_timeout = Math.floor((settle_time + 300) / time_conversion);
  const settle_strength = Math.floor(motor.PWM_BASE * 2 / 10);

  const drive_options = {command_timeout: settle_timeout, command_value: settle_strength};
  const test_options = {command_timeout: 0, command_value: 0};

  console.info("Current calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_INCREASING, ...test_options});

  const u_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("U positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_DECREASING, ...test_options});

  const w_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("W negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_INCREASING, ...test_options});

  const v_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("V positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_DECREASING, ...test_options});

  const u_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("U negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_POSITIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_INCREASING, ...test_options});

  const w_positive = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("W positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_NEGATIVE, ...drive_options});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_DECREASING, ...test_options});

  const v_negative = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE})).data;

  console.info("V negative done");

  // Check all calibration data is complete.
  if (u_positive.length !== motor.HISTORY_SIZE) {
    console.error("U positive calibration data incomplete", u_positive);
    return;
  }
  if (u_negative.length !== motor.HISTORY_SIZE) {
    console.error("U negative calibration data incomplete", u_negative);
    return;
  }
  if (v_positive.length !== motor.HISTORY_SIZE) {
    console.error("V positive calibration data incomplete", v_positive);
    return;
  }
  if (v_negative.length !== motor.HISTORY_SIZE) {
    console.error("V negative calibration data incomplete", v_negative);
    return;
  }
  if (w_positive.length !== motor.HISTORY_SIZE) {
    console.error("W positive calibration data incomplete", w_positive);
    return;
  }
  if (w_negative.length !== motor.HISTORY_SIZE) {
    console.error("W negative calibration data incomplete", w_negative);
    return;
  }

  // Make a new table with each calibration phase as a column.
  const calibration_data = d3.range(motor.HISTORY_SIZE).map((i) => {
    return {
      time: u_positive[i].time,
      u_positive: u_positive[i].u_readout,
      u_negative: -u_negative[i].u_readout,
      v_positive: v_positive[i].v_readout,
      v_negative: -v_negative[i].v_readout,
      w_positive: w_positive[i].w_readout,
      w_negative: -w_negative[i].w_readout,
    };
  });

  console.info("Current calibration done");
  
  store_current_calibration_results(calibration_data);
}

```


```js


const current_calibration_stats = d3.range(motor.HISTORY_SIZE).map((i) => {
  return {
    time: current_calibration_results[0][i].time,
    u_positive_mean: d3.mean(current_calibration_results, (data) => data[i].u_positive),
    u_positive_std: d3.deviation(current_calibration_results, (data) => data[i].u_positive),
    u_negative_mean: d3.mean(current_calibration_results, (data) => data[i].u_negative),
    u_negative_std: d3.deviation(current_calibration_results, (data) => data[i].u_negative),
    v_positive_mean: d3.mean(current_calibration_results, (data) => data[i].v_positive),
    v_positive_std: d3.deviation(current_calibration_results, (data) => data[i].v_positive),
    v_negative_mean: d3.mean(current_calibration_results, (data) => data[i].v_negative),
    v_negative_std: d3.deviation(current_calibration_results, (data) => data[i].v_negative),
    w_positive_mean: d3.mean(current_calibration_results, (data) => data[i].w_positive),
    w_positive_std: d3.deviation(current_calibration_results, (data) => data[i].w_positive),
    w_negative_mean: d3.mean(current_calibration_results, (data) => data[i].w_negative),
    w_negative_std: d3.deviation(current_calibration_results, (data) => data[i].w_negative),
  };
});

const current_calibration_targets = current_calibration_zones.map((zone) => zone.pwm * calibration_reference);


function compute_current_calibration(calibration_data){

  function compute_phase_calibration(phase_selector){

    function select_by_zone(calibration_data){
      return current_calibration_zones.map((zone) => {
        return calibration_data.filter((d) => d.time > zone.settle_start && d.time < zone.settle_end);
      });
    }
  
    function compute_zone_calibration({measurements, targets}){
      // Make sure the lengths are equal.
      if (measurements.length !== targets.length) throw new Error("Data length mismatch");
  
      const factor = d3.mean(targets, (target, i) => target / measurements[i]); 
      
      const slow_calibration = piecewise_linear({
        X: [0.0, ...measurements.map((x) => x)], 
        Y: [0.0, ...targets.map((y) => y / factor)],
      });
  
      // Recalibrate to evenly spaced points for fast processing.

      const Y = current_calibration_reading_points.map((x) => slow_calibration(x));
  
  
      const func = even_piecewise_linear({x_min: 0, x_max: calibration_reference, Y});
  
      const sample = current_calibration_reading_points.map((x) => ({reading: x, target: func(x)}));
      
      return {
        measurements,
        targets,
        factor,
        func,
        sample,
      };
    }
  
    return compute_zone_calibration({
      measurements: select_by_zone(calibration_data).map((zone_data) => d3.mean(zone_data, (d) => d[phase_selector])),
      targets: current_calibration_targets,
    });
  }

  return {
    u_positive: compute_phase_calibration("u_positive_mean"),
    u_negative: compute_phase_calibration("u_negative_mean"),
    v_positive: compute_phase_calibration("v_positive_mean"),
    v_negative: compute_phase_calibration("v_negative_mean"),
    w_positive: compute_phase_calibration("w_positive_mean"),
    w_negative: compute_phase_calibration("w_negative_mean"),
  };
}

const current_calibration = compute_current_calibration(current_calibration_stats); 

update_current_calibration(current_calibration);

const current_calibration_table = html`<div>Phase correction factors:</div><table>
  <tr><td>U:</td><td>${current_calibration.u_positive.factor.toFixed(3)}</td><td>${current_calibration.u_negative.factor.toFixed(3)}</td></tr>
  <tr><td>V:</td><td>${current_calibration.v_positive.factor.toFixed(3)}</td><td>${current_calibration.v_negative.factor.toFixed(3)}</td></tr>
  <tr><td>W:</td><td>${current_calibration.w_positive.factor.toFixed(3)}</td><td>${current_calibration.w_negative.factor.toFixed(3)}</td></tr>
</table>`;

const current_calibration_result_to_display_input = Inputs.select(d3.range(current_calibration_results.length), {
  value: current_calibration_results.length - 1,
  label: "Select calibration result to display:",
});
const current_calibration_result_to_display = Generators.input(current_calibration_result_to_display_input);

```
```js

const current_calibration_plot = plot_multiline({
  data: current_calibration_results[current_calibration_result_to_display],
  store_id: "plot_current_calibration",
  selection: null,
  subtitle: "Current Calibration",
  description: "Current calibration results for each phase.",
  width: 1200, height: 400,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {},
  x: "time",
  y: "u_positive",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channel_label: "Phase",
  channels: [
    {y: "u_positive", label: "U positive", color: colors.u},
    {y: "u_negative", label: "U negative", color: d3.color(colors.u).darker(1)},
    {y: "v_positive", label: "V positive", color: colors.v},
    {y: "v_negative", label: "V negative", color: d3.color(colors.v).darker(1)},
    {y: "w_positive", label: "W positive", color: colors.w},
    {y: "w_negative", label: "W negative", color: d3.color(colors.w).darker(1)},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    Plot.rect(current_calibration_zones, {x1: "settle_start", x2: "settle_end", y1: 0, y2: (zone) => zone.pwm * calibration_reference, fill: "rgba(0, 0, 0, 0.05)"}),
  ]
});

const calibration_samples = current_calibration_reading_points.map((x, i) => {
  return {
    reading: x,
    target: x,
    u_positive: current_calibration.u_positive.sample[i].target,
    u_negative: current_calibration.u_negative.sample[i].target,
    v_positive: current_calibration.v_positive.sample[i].target,
    v_negative: current_calibration.v_negative.sample[i].target,
    w_positive: current_calibration.w_positive.sample[i].target,
    w_negative: current_calibration.w_negative.sample[i].target,
  };
});

const current_calibration_interpolate_plot = plot_multiline({
  data: calibration_samples,
  store_id: "plot_current_calibration_interpolate",
  selection: null,
  subtitle: "Current Calibration Interpolation",
  description: "Current calibration interpolation results for each phase.",
  width: 1200, height: 400,
  x_options: {domain: [0, calibration_reference]},
  y_options: {},
  x: "reading",
  y: "u_positive",
  x_label: "Current Reading (A)",
  y_label: "Current Estimate (A)",
  channel_label: "Phase",
  channels: [
    {y: "u_positive", label: "U positive", color: colors.u},
    {y: "u_negative", label: "U negative", color: d3.color(colors.u).darker(1)},
    {y: "v_positive", label: "V positive", color: colors.v},
    {y: "v_negative", label: "V negative", color: d3.color(colors.v).darker(1)},
    {y: "w_positive", label: "W positive", color: colors.w},
    {y: "w_negative", label: "W negative", color: d3.color(colors.w).darker(1)},
    {y: "target", label: "Target", color: "gray"},
  ],
  grid_marks: [
    Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.1, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
});


const current_calibration_positive_mean_plot = plot_multiline({
  data: current_calibration_stats,
  store_id: "plot_current_calibration_positive_mean",
  selection: null,
  subtitle: "Current Calibration Mean",
  description: "Current calibration mean results for each phase driven positive.",
  width: 1200, height: 300,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {},
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channel_label: "Phase",
  channels: [
    {
      y: "u_positive_mean", 
      y1: (d) => d.u_positive_mean - d.u_positive_std * std_95_z_score, 
      y2: (d) => d.u_positive_mean + d.u_positive_std * std_95_z_score, 
      label: "U positive mean", color: colors.u,
    },
    {
      y: "v_positive_mean",
      y1: (d) => d.v_positive_mean - d.v_positive_std * std_95_z_score,
      y2: (d) => d.v_positive_mean + d.v_positive_std * std_95_z_score, 
      label: "V positive mean", color: colors.v,
    },
    {
      y: "w_positive_mean",
      y1: (d) => d.w_positive_mean - d.w_positive_std * std_95_z_score,
      y2: (d) => d.w_positive_mean + d.w_positive_std * std_95_z_score,
      label: "W positive mean", color: colors.w,
    },
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    Plot.rect(current_calibration_zones, {x1: "settle_start", x2: "settle_end", y1: 0, y2: (zone) => zone.pwm * calibration_reference, fill: "rgba(0, 0, 0, 0.05)"}),
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ]
});

const current_calibration_negative_mean_plot = plot_multiline({
  data: current_calibration_stats,
  store_id: "plot_current_calibration_negative_mean",
  selection: null,
  subtitle: "Current Calibration Mean",
  description: "Current calibration mean results for each phase driven negative.",
  width: 1200, height: 300,
  x_options: {domain: [0, motor.HISTORY_SIZE * time_conversion]},
  y_options: {},
  x: "time",
  x_label: "Time (ms)",
  y_label: "Current (A)",
  channel_label: "Phase",
  channels: [
    {
      y: "u_negative_mean", 
      y1: (d) => d.u_negative_mean - d.u_negative_std * std_95_z_score, 
      y2: (d) => d.u_negative_mean + d.u_negative_std * std_95_z_score, 
      label: "U negative mean", color: colors.u,
    },
    {
      y: "v_negative_mean",
      y1: (d) => d.v_negative_mean - d.v_negative_std * std_95_z_score,
      y2: (d) => d.v_negative_mean + d.v_negative_std * std_95_z_score, 
      label: "V negative mean", color: colors.v,
    },
    {
      y: "w_negative_mean",
      y1: (d) => d.w_negative_mean - d.w_negative_std * std_95_z_score,
      y2: (d) => d.w_negative_mean + d.w_negative_std * std_95_z_score,
      label: "W negative mean", color: colors.w,
    },
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ],
  other_marks: [
    Plot.rect(current_calibration_zones, {x1: "settle_start", x2: "settle_end", y1: 0, y2: (zone) => zone.pwm * calibration_reference, fill: "rgba(0, 0, 0, 0.05)"}),
    (selected_data, options) => Plot.areaY(selected_data, {...options, y1: "y1", y2: "y2", fill: options.z, opacity: 0.2}),
  ]
});

```



```js
// Imports
// -------

import {plot_multiline, horizontal_step} from "./components/plotting_utils.js";
import {localStorage, get_stored_or_default} from "./components/local_storage.js";
import {round, uint32_to_bytes, bytes_to_uint32, timeout_promise, wait, clean_id}  from "./components/utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./components/math_utils.js";
import * as motor from "./components/usb_motor_controller.js";

```

