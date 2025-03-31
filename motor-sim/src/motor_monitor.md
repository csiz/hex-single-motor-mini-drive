---
title: Motor monitor
---


Motor Command Dashboard
-----------------------

<div>${connect_buttons}</div>
<div>${motor_controller_status}</div>
<div>${data_stream_buttons}</div>
<div>${command_buttons}</div>
<div>
  ${command_value_slider}
  ${command_timeout_slider}
</div>


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

const max_calibration_current = 0.9 * drive_voltage / drive_resistance;

const phase_inductance = 0.000_145; // 290 uH measured with LCR meter across phase pairs.
const phase_resistance = 2.0; // 2.0 Ohm

const π = Math.PI;


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
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0});
    }],
    ["Freewheel Snapshot", async function(){
      await command(motor.SET_STATE_FREEWHEEL);
      await command_and_stream(motor.GET_READOUTS_SNAPSHOT, {expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0});
    }],
    ["ADC stream", async function(){
      await command_and_stream(motor.GET_READOUTS, {max_missed_messages: 128});
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


const identity_calibration_factors = {
  u_positive: 1.0,
  u_negative: 1.0,
  v_positive: 1.0,
  v_negative: 1.0,
  w_positive: 1.0,
  w_negative: 1.0,
};

function load_calibration_factors(){
  return get_stored_or_default("current_calibration", identity_calibration_factors);
}

// Load or make default calibration
const saved_calibration_factors = load_calibration_factors();

let calibration_factors = Mutable(saved_calibration_factors);

function update_calibration_factors(new_calibration_factors){
  calibration_factors.value = new_calibration_factors;
}

function reload_calibration_factors(){
  update_calibration_factors(load_calibration_factors());
}

function save_calibration_factors(){
  localStorage.setItem("current_calibration", JSON.stringify(calibration_factors.value));
}

function reset_calibration_factors(){
  localStorage.removeItem("current_calibration");
  update_calibration_factors(identity_calibration_factors);
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

function calculate_data_stats(raw_readout_data){

  const ref_readout_mean = d3.mean(raw_readout_data, (d) => d.ref_readout);
  const start_readout_number = raw_readout_data[0].readout_number;


  const data_with_annotations = raw_readout_data.map((d) => {
    const readout_number = d.readout_number - start_readout_number;
    const time = readout_number * time_conversion;
  
    const u_readout = -current_conversion * (d.u_readout - d.ref_readout);
    const v_readout = +current_conversion * (d.v_readout - d.ref_readout);
    const w_readout = -current_conversion * (d.w_readout - d.ref_readout);
    const ref_diff = current_conversion * (d.ref_readout - ref_readout_mean);
  
    const calibrated_u = u_readout * (u_readout >= 0 ? 
      calibration_factors.u_positive :
      calibration_factors.u_negative);

    const calibrated_v = v_readout * (v_readout >= 0 ?
      calibration_factors.v_positive :
      calibration_factors.v_negative);

    const calibrated_w = w_readout * (w_readout >= 0 ?
      calibration_factors.w_positive :
      calibration_factors.w_negative);

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
    const left_derivative = df(left, mid) / (mid.time - left.time) * 1000;
    const right_derivative = df(mid, right) / (right.time - mid.time) * 1000;
    return (left_derivative + right_derivative) / 2.0;
  }

  function inductor_voltage_3_points(index, df, data){
    const derivative = derivative_3_points(index, df, data);
    return derivative == null ? null : derivative * phase_inductance; // V = L*dI/dt + R*I
  }

  function diff_current_angle(a, b){
    const diff = (b.current_angle - a.current_angle) * Math.PI / 180.0;
    return diff > Math.PI ? diff - 2 * Math.PI : diff < -Math.PI ? diff + 2 * Math.PI : diff;
  }

  function speed_3_points(index, df, data){
    const derivative = derivative_3_points(index, df, data);
    return derivative == null ? null : derivative / (1000 * 2 * Math.PI); // rotations per millisecond
  }

  const data_with_derivatives = data_with_annotations.map((d, i, data) => {
    const radial_speed = speed_3_points(i, diff_current_angle, data);
    const u_L_voltage = inductor_voltage_3_points(i, (a, b) => b.u - a.u, data);
    const v_L_voltage = inductor_voltage_3_points(i, (a, b) => b.v - a.v, data);
    const w_L_voltage = inductor_voltage_3_points(i, (a, b) => b.w - a.w, data);
    
    const u_voltage = u_L_voltage === null ? null : u_L_voltage + phase_resistance * d.u;
    const v_voltage = v_L_voltage === null ? null : v_L_voltage + phase_resistance * d.v;
    const w_voltage = w_L_voltage === null ? null : w_L_voltage + phase_resistance * d.w;

    const any_null = radial_speed === null || u_L_voltage === null || v_L_voltage === null || w_L_voltage === null;

    const [voltage_alpha, voltage_beta] = any_null ? [null, null] : matrix_multiply(power_invariant_clarke_matrix, [u_voltage, v_voltage, w_voltage]);
    const voltage_angle = any_null ? null : Math.atan2(voltage_beta, voltage_alpha);
    
    const voltage_magnitude = any_null ? null : Math.sqrt(voltage_alpha * voltage_alpha + voltage_beta * voltage_beta);

    const breaking_angle = any_null ? null : ((voltage_angle + (radial_speed > 0 ? Math.PI / 2.0 : -Math.PI / 2.0)) + 3 * Math.PI) % (2 * Math.PI) - Math.PI;

    return {
      ...d,
      radial_speed,
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
      breaking_angle: breaking_angle === null ? null : breaking_angle * 180 / Math.PI,
    };
  });

  const data = data_with_derivatives;

  return {data, ref_readout_mean, start_readout_number};
}

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
  current_magnitude: "rgb(197, 152, 67)",
  current_angle: "rgb(102, 166, 30)",
  radial_speed: "rgb(27, 158, 119)",
  current_alpha: "rgb(199, 0, 57)",
  current_beta: "rgb(26, 82, 118)",
};


function tidy_select({data, x, x_label = "x", y_label = "y", channel_label = "channel", channels}){
  return data.flatMap((d) => {
    return channels.map(({y, label}) => {
      return Object.fromEntries([
        [x_label, d[x]],
        [y_label, d[y]],
        [channel_label, label],
      ]);
    });
  });
}

function plot_multiline(options){
  const {
    data, 
    store_id,
    width, height,
    x_options, y_options,
    x, y, 
    x_label, y_label,
    channel_label, channels,
    subtitle, description,
    grid_marks = [],
    curve = undefined,
  } = options;

  let {selection} = options;

  selection = selection ?? get_stored_or_default(store_id, {
    show: true,
    shown_marks: [...channels.map(({y}) => y), "grid"],
  });

  let result = Mutable(create_element(selection));

  function update_selection(new_selection){
    selection = {...selection, ...new_selection};
    // Store the selection in local storage.
    localStorage.setItem(store_id, JSON.stringify(selection));
    // Update the plot.
    result.value = create_element(selection);
  }

  function create_element(selection){

    // First, make the title into a checkbox to toggle the plot on and off.
    const subtitle_checkbox = Inputs.checkbox([subtitle], {
      value: selection.show ? [subtitle] : [],
      format: (subtitle) => html`<h4 style="min-width: 20em; font-size: 1.5em; font-weight: normal;">${subtitle}</h4>`,
    });

    subtitle_checkbox.addEventListener("input", function(event){
      const show = subtitle_checkbox.value.length > 0;
      update_selection({show});
    });

    if (!selection.show) {
      return html`<div>${subtitle_checkbox}</div>`;
    }

    const description_element = html`<p>${description}</p>`;

    const checkbox_y_to_label = Object.fromEntries([...channels.map(({y, label}) => [y, label]), ["grid", "Grid"]]);
    const checkbox_y_to_color = Object.fromEntries([...channels.map(({y, color}) => [y, color]), ["grid", "grey"]]);

    // Then, make the marks into checkboxes to toggle them on and off.
    const marks_checkboxes = Inputs.checkbox(
      [...channels.map(({y}) => y), "grid"], 
      {
        value: selection.shown_marks,
        label: "Display:",
        format: (y) => html`<span style="border-bottom: solid 3px ${checkbox_y_to_color[y]}; margin-bottom: -3px;">${checkbox_y_to_label[y]}</span>`,
      },
    );

    marks_checkboxes.addEventListener("input", function(event){
      const shown_marks = marks_checkboxes.value;
      update_selection({shown_marks});
    });

    const selected_channels = channels.filter(({y}) => selection.shown_marks.includes(y));
    const selected_data = tidy_select({data, x, x_label, y_label, channel_label, channels: selected_channels});

    const plot_figure = Plot.plot({
      width, height,
      x: {label: x_label, ...x_options},
      y: {label: y_label, domain: selected_data.length > 0 ? undefined : [0, 1], ...y_options},
      color: {
        // legend: true,
        domain: channels.map(({label}) => label),
        range: channels.map(({color}) => color),
      },
      marks: [
        Plot.line(selected_data, {x: x_label, y: y_label, stroke: channel_label, curve}),
        Plot.crosshairX(selected_data, {x: x_label, y: y_label, color: channel_label, ruleStrokeWidth: 3}),
        Plot.dot(selected_data, Plot.pointerX({x: x_label, y: y_label, stroke: channel_label})),
        Plot.text(
          selected_data,
          Plot.pointerX({
            px: x_label, py: y_label, fill: channel_label,
            dy: -17, frameAnchor: "top-right", monospace: true, fontSize: 14, fontWeight: "bold",
            text: (d) => `Channel: ${d[channel_label].padEnd(20)} | ${x_label}: ${d[x_label]?.toFixed(3).padStart(9)} | ${y_label}: ${d[y_label]?.toFixed(3).padStart(9)}`,
          }),
        ),
        ...(selection.shown_marks.includes("grid") ? grid_marks : []),
      ],
    });

    return html`<div>${subtitle_checkbox}${description_element}${marks_checkboxes}${plot_figure}</div>`;
  }

  return result;
}

```


Motor Driving Data
------------------
<div class="card tight">
Controls for the plotting time window:
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
    {y: "current_angle", label: "Current (Park) Angle", color: colors.current_angle},
    {y: "voltage_angle", label: "Voltage (Park) Angle", color: d3.color(colors.current_angle).darker(1)},
    {y: "breaking_angle", label: "Inferred Angle", color: d3.color(colors.current_angle).darker(2)},
    {y: "hall_u_as_angle", label: "Hall U", color: colors.u},
    {y: "hall_v_as_angle", label: "Hall V", color: colors.v},
    {y: "hall_w_as_angle", label: "Hall W", color: colors.w},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({interval: 90, stroke: 'black', strokeWidth : 2}),
  ],
  curve: plot_options.includes("Connected lines") ? "step" : horizontal_step,
});

const plot_speed = plot_multiline({
  data: selected_data,
  store_id: "plot_speed",
  selection: null,
  subtitle: "Rotor Speed",
  description: "Angular speed of the rotor in rotations per millisecond.",
  width: 1200, height: 150,
  x_options: {domain: time_domain},
  y_options: {},
  x: "time",
  y: "radial_speed",
  x_label: "Time (ms)",
  y_label: "Speed (rotations/ms)",
  channel_label: "Speed Source",
  channels: [
    {y: "radial_speed", label: "Speed", color: colors.radial_speed},
  ],
  grid_marks: [
    Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 2}),
    Plot.gridX({interval: 0.2, stroke: 'black', strokeWidth : 1}),
    Plot.gridY({stroke: 'black', strokeWidth : 2}),
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

Calibration Procedures
----------------------

<div class="card tight">
  <div>${calibration_buttons}</div>
  <div>${calibration_plots}</div>
</div>

```js
let calibration_results = Mutable([]);

function store_calibration_result(calibration_data){
  calibration_results.value = [calibration_data, ...calibration_results.value];
}
```


```js
const short_duration = motor.HISTORY_SIZE / 12 * time_conversion;

const calibration_zones = [
  {pwm: 0.1, start: short_duration * 1.4, end: short_duration * 1.95},
  {pwm: 0.2, start: short_duration * 2.4, end: short_duration * 2.95},
  {pwm: 0.3, start: short_duration * 3.4, end: short_duration * 3.95},
  {pwm: 0.4, start: short_duration * 4.4, end: short_duration * 4.95},
  {pwm: 0.5, start: short_duration * 5.4, end: short_duration * 5.95},
  {pwm: 0.6, start: short_duration * 6.4, end: short_duration * 6.95},
  {pwm: 0.7, start: short_duration * 7.4, end: short_duration * 7.95},
  {pwm: 0.8, start: short_duration * 8.4, end: short_duration * 8.95},
  {pwm: 0.9, start: short_duration * 9.4, end: short_duration * 9.95},
];


const calibration_points = 32;




async function run_calibration(){
  if (!motor_controller) return;
  
  const settle_time = 300;
  const settle_timeout = settle_time * 3;
  const settle_strength = Math.floor(motor.PWM_BASE * 2 / 10);

  console.info("Calibration starting");

  // Note: hold pwm is clamped by the motor driver

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_INCREASING, command_timeout: 0, command_value: 0});

  const u_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_DECREASING, command_timeout: 0, command_value: 0});

  const w_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_INCREASING, command_timeout: 0, command_value: 0});

  const v_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("V positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_U_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_U_DECREASING, command_timeout: 0, command_value: 0});

  const u_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("U negative done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_W_POSITIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_W_INCREASING, command_timeout: 0, command_value: 0});

  const w_positive_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("W positive done");

  await motor_controller.send_command({command: motor.SET_STATE_HOLD_V_NEGATIVE, command_timeout: settle_timeout, command_value: settle_strength});
  await wait(settle_time);
  await motor_controller.send_command({command: motor.SET_STATE_TEST_V_DECREASING, command_timeout: 0, command_value: 0});

  const v_negative_data = calculate_data_stats(await motor_controller.get_readouts({expected_messages: motor.HISTORY_SIZE, max_missed_messages: 0})).data;

  console.info("V negative done");

  const calibration_data = {
    u_positive_data,
    u_negative_data,
    v_positive_data,
    v_negative_data,
    w_positive_data,
    w_negative_data,
  };

  console.info("Calibration done");
  
  store_calibration_result(calibration_data);
}



function compute_zone_calibration({measurements, targets}){
  // Make sure the lengths are equal.
  if (measurements.length !== targets.length) throw new Error("Data length mismatch");

  const factor = d3.mean(targets, (target, i) => target / measurements[i]); 

  const slow_calibration = piecewise_linear({
    X: [0.0, ...measurements.map((x) => x)], 
    Y: [0.0, ...targets.map((y) => y / factor)],
  });

  const n = calibration_points / 2 + 1;

  // Recalibrate to evenly spaced points for fast processing.

  const X = even_spacing(max_calibration_current, n);
  const Y = X.map((x) => slow_calibration(x));


  const func = even_piecewise_linear({x_min: 0, x_max: max_calibration_current, Y});

  const sample = X.map((x) => ({reading: x, target: func(x)}));
  
  return {
    measurements,
    targets,
    factor,
    func,
    sample,
  };
}


function compute_calibration_stats(data_selector, readout_selector){
  if (calibration_results.length < 1) return null;

  const phase_calibration_data = calibration_results.map(data_selector);
  // Check the length of the data is the same for all collections.
  const latest_data = phase_calibration_data[0];
  if (phase_calibration_data.some((d) => d.length !== latest_data.length)) {
    console.error(phase_calibration_data.map((d) => d.length));
    throw new Error("Data length mismatch");
  }
  
  return latest_data.map((d, i) => {
    const calibration_data = calibration_results.map(data_selector).map((d) => d[i]);
    const readouts = calibration_data.map(readout_selector);
    return {
      time: d.time,
      readout_latest: readout_selector(d),
      readout_mean: d3.mean(readouts),
      readout_std: d3.deviation(readouts),
    };
  });
}

const calibration_stats = {
  u_positive_stats: compute_calibration_stats((d) => d.u_positive_data, (d) => d.u_readout),
  u_negative_stats: compute_calibration_stats((d) => d.u_negative_data, (d) => -d.u_readout),
  v_positive_stats: compute_calibration_stats((d) => d.v_positive_data, (d) => d.v_readout),
  v_negative_stats: compute_calibration_stats((d) => d.v_negative_data, (d) => -d.v_readout),
  w_positive_stats: compute_calibration_stats((d) => d.w_positive_data, (d) => d.w_readout),
  w_negative_stats: compute_calibration_stats((d) => d.w_negative_data, (d) => -d.w_readout),
};

const predicted_targets = calibration_zones.map((zone) => zone.pwm * drive_voltage / drive_resistance);

function select_by_zone(data){
  return calibration_zones.map((zone) => {
    return data.filter((d) => d.time > zone.start && d.time < zone.end);
  });
}

function compute_phase_calibration(data){
  if (!data) return null;

  return compute_zone_calibration({
    measurements: select_by_zone(data).map((zone_data) => d3.mean(zone_data, (d) => d.readout_mean)),
    targets: predicted_targets,
  });
}

const calibration = {
  u_positive: compute_phase_calibration(calibration_stats.u_positive_stats),
  u_negative: compute_phase_calibration(calibration_stats.u_negative_stats),
  v_positive: compute_phase_calibration(calibration_stats.v_positive_stats),
  v_negative: compute_phase_calibration(calibration_stats.v_negative_stats),
  w_positive: compute_phase_calibration(calibration_stats.w_positive_stats),
  w_negative: compute_phase_calibration(calibration_stats.w_negative_stats),
};


const calibration_plots = [
  html`<div>Number of calibration data sets: ${calibration_results.length}</div>`,
  ...(calibration_results.length < 1 ? [html`<div>No calibration data</div>`] : [
    html`<h3>Calibration results</h3>`,
    Plot.plot({
      marks: [
        Plot.line(calibration_stats.u_positive_stats, {x: "time", y: "readout_latest", stroke: colors.u, label: "u positive"}),
        Plot.line(calibration_stats.v_positive_stats, {x: "time", y: "readout_latest", stroke: colors.v, label: "v positive"}),
        Plot.line(calibration_stats.w_positive_stats, {x: "time", y: "readout_latest", stroke: colors.w, label: "w positive"}),
        Plot.line(calibration_stats.u_negative_stats, {x: "time", y: "readout_latest", stroke: colors.u, strokeDasharray: "2 5", label: "u negative"}),
        Plot.line(calibration_stats.v_negative_stats, {x: "time", y: "readout_latest", stroke: colors.v, strokeDasharray: "2 5", label: "v negative"}),
        Plot.line(calibration_stats.w_negative_stats, {x: "time", y: "readout_latest", stroke: colors.w, strokeDasharray: "2 5", label: "w negative"}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: 0, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 400,
    }),
    html`<div>Phase correction factors:</div><table>
      <tr><td>U:</td><td>${calibration.u_positive.factor.toFixed(3)}</td><td>${calibration.u_negative.factor.toFixed(3)}</td></tr>
      <tr><td>V:</td><td>${calibration.v_positive.factor.toFixed(3)}</td><td>${calibration.v_negative.factor.toFixed(3)}</td></tr>
      <tr><td>W:</td><td>${calibration.w_positive.factor.toFixed(3)}</td><td>${calibration.w_negative.factor.toFixed(3)}</td></tr>
    </table>`,
    Plot.plot({
      marks: [
        Plot.line([[0, 0], [max_calibration_current, max_calibration_current]], {stroke: "gray", label: "reference"}),
        Plot.line(calibration.u_positive.sample, {x: "reading", y: "target", stroke: colors.u, label: "u", marker: "circle"}),
        Plot.line(calibration.v_positive.sample, {x: "reading", y: "target", stroke: colors.v, label: "v", marker: "circle"}),
        Plot.line(calibration.w_positive.sample, {x: "reading", y: "target", stroke: colors.w, label: "w", marker: "circle"}),
        Plot.line(calibration.u_negative.sample, {x: "reading", y: "target", stroke: colors.u, strokeDasharray: "2 5", label: "u linear", marker: "circle"}),
        Plot.line(calibration.v_negative.sample, {x: "reading", y: "target", stroke: colors.v, strokeDasharray: "2 5", label: "v linear", marker: "circle"}),
        Plot.line(calibration.w_negative.sample, {x: "reading", y: "target", stroke: colors.w, strokeDasharray: "2 5", label: "w linear", marker: "circle"}),
        Plot.gridX({interval: 0.5, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Current Reading (A)"},
      y: {label: "Current Estimate (A)"},
      width: 1200, height: 400,
    }),
  ]),
  ...(calibration_results.length < 2 ? [html`<div>No calibration statistics</div>`] : [
    html`<h3>Calibration statistics</h3>`,
    Plot.plot({
      marks: [
        Plot.line(calibration_stats.u_positive_stats, {x: "time", y: "readout_mean", stroke: colors.u, label: "u positive"}),
        Plot.line(calibration_stats.v_positive_stats, {x: "time", y: "readout_mean", stroke: colors.v, label: "v positive"}),
        Plot.line(calibration_stats.w_positive_stats, {x: "time", y: "readout_mean", stroke: colors.w, label: "w positive"}),
        Plot.line(calibration_stats.u_negative_stats, {x: "time", y: "readout_mean", stroke: colors.u, strokeDasharray: "2 5", label: "u negative"}),
        Plot.line(calibration_stats.v_negative_stats, {x: "time", y: "readout_mean", stroke: colors.v, strokeDasharray: "2 5", label: "v negative"}),
        Plot.line(calibration_stats.w_negative_stats, {x: "time", y: "readout_mean", stroke: colors.w, strokeDasharray: "2 5", label: "w negative"}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: 0, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 400,
    }),
    Plot.plot({
      marks: [
        Plot.areaY(calibration_stats.u_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.u}),
        Plot.areaY(calibration_stats.v_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.v}),
        Plot.areaY(calibration_stats.w_positive_stats, {x: "time", y1: (d) => d.readout_mean - d.readout_std, y2: (d) => d.readout_mean + d.readout_std, fill: colors.w}),
        Plot.areaY(calibration_stats.u_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.u}),
        Plot.areaY(calibration_stats.v_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.v}),
        Plot.areaY(calibration_stats.w_negative_stats, {x: "time", y1: (d) => -d.readout_mean - d.readout_std, y2: (d) => -d.readout_mean + d.readout_std, fill: colors.w}),
        Plot.rect(calibration_zones, {x1: "start", x2: "end", y1: -max_calibration_current, y2: max_calibration_current, fill: "rgba(0, 0, 0, 0.05)"}),
        Plot.gridX({interval: 1.0, stroke: 'black', strokeWidth : 1}),
        Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 1}),
      ],
      x: {label: "Time (ms)"},
      y: {label: "Current (A)"},
      width: 1200, height: 600,
    }),
  ]),
];



const calibration_buttons = Inputs.button(
  [
    ["Start Calibration", async function(){
      await run_calibration();
    }],
    ["Save Calibration", function(){
      update_calibration_factors({
        u_positive: calibration.u_positive.factor,
        u_negative: calibration.u_negative.factor,
        v_positive: calibration.v_positive.factor,
        v_negative: calibration.v_negative.factor,
        w_positive: calibration.w_positive.factor,
        w_negative: calibration.w_negative.factor,
      });
      save_calibration_factors();
    }],
    ["Reload Calibration", function(){
      reload_calibration_factors();
    }],
    ["Reset Calibration", function(){
      reset_calibration_factors();
    }],
  ],
  {label: "Collect calibration data"},
);

```



```js
function HorizontalStep(context, t) {
  this._context = context;
  this._t = t;
}

HorizontalStep.prototype = {
  areaStart: function() {
    this._line = 0;
  },
  areaEnd: function() {
    this._line = NaN;
  },
  lineStart: function() {
    this._x = this._y = NaN;
    this._point = 0;
  },
  lineEnd: function() {
    if (0 < this._t && this._t < 1 && this._point === 2) this._context.lineTo(this._x, this._y);
    if (this._line || (this._line !== 0 && this._point === 1)) this._context.closePath();
    if (this._line >= 0) this._t = 1 - this._t, this._line = 1 - this._line;
  },
  point: function(x, y) {
    x = +x, y = +y;
    switch (this._point) {
      case 0: this._point = 1; this._line ? this._context.lineTo(x, y) : this._context.moveTo(x, y); break;
      case 1: this._point = 2; // falls through
      default: {
        if (this._t <= 0) {
          this._context.lineTo(this._x, y);
          this._context.lineTo(x, y);
        } else {
          var x1 = this._x * (1 - this._t) + x * this._t;
          this._context.lineTo(x1, this._y);
          this._context.moveTo(x1, y);
        }
        break;
      }
    }
    this._x = x, this._y = y;
  }
};

function horizontal_step(context) {
  return new HorizontalStep(context, 0.5);
}

function horizontal_step_before(context) {
  return new HorizontalStep(context, 0);
}

function horizontal_step_after(context) {
  return new HorizontalStep(context, 1);
}
```



```js
// Imports
// -------

import {html} from "htl";
import {localStorage, get_stored_or_default} from "./components/local_storage.js";
import {round, uint32_to_bytes, bytes_to_uint32, timeout_promise, wait, clean_id}  from "./components/utils.js";
import {even_spacing, piecewise_linear, even_piecewise_linear} from "./components/math_utils.js";
import * as motor from "./components/usb_motor_controller.js";

```