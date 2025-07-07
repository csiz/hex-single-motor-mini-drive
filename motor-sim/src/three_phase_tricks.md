---
title: 3 Phase Tricks
---

<main class="hero">

3 Phase Tricks
--------------


The voltages of the 3 phases are normally sinusoidal but the important metric is the
voltage difference between the phases. Notice that the minimum and maximum voltages
don't occur at the same time for all phases.


<div class="card tight">
    <div>${phases_and_maximums_plot}</div>
    <div>${phases_difference_plot}</div>
</div>

A convenient trick is to adjust the phases so that the minimum voltage is always zero
for some phase. The phase voltages below are equivalent to those above. Notice that
we use 86% of the voltage range after the adjustment. We can thus scale our voltages
by 1.16 in order to use the full range and get 15% more effective voltage (in the 
rotating phasor frame).

<div class="card tight">
  <div>${phases_rebased_plot}</div>
</div>

We need to approximate the atan2 function because we don't have enough processing to
compute it in real time.

<div class="card tight">
  <div>${atan_approx_plot}</div>
</div>

<pre>${phases_wave_form_lookup_table}</pre>
<pre>${sin_lookup_table}</pre>
</main>


```js

import {angle_base, pwm_base} from "./components/motor_constants.js";

const phi = d3.range(angle_base).map(x => 2 * Math.PI * x / angle_base);
function phase_func(t){
  const deg = t * 180 / Math.PI;

  const sin = Math.sin(t);

  const u = Math.cos(t);
  const v = Math.cos(t + 2 * Math.PI / 3);
  const w = Math.cos(t + 4 * Math.PI / 3);

  const uv = u - v;
  const vw = v - w;
  const wu = w - u;

  const max = Math.max(u, v, w);
  const min = Math.min(u, v, w);
  const max_adj = max;
  const min_adj = min;
  const adj = - min_adj;

  const adj_u = u + adj;
  const adj_v = v + adj;
  const adj_w = w + adj;
  return {t, sin, deg, u, v, w, adj: -adj, max, min, max_adj, min_adj, adj_u, adj_v, adj_w, uv, vw, wu};
}

const phases = phi.map(phase_func);

const max_adj = d3.max(phases, d => d.adj_u);

const phases_and_maximums_plot = Plot.plot({
  y: {domain: [-1.0, 1.0]},
  marks: [
    Plot.lineY(phases, {x: "deg", y: "u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: "deg", y: "v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: "deg", y: "w", stroke: 'blue', label: 'W'}),
    Plot.lineY(phases, {x: "deg", y: "max_adj", stroke: 'black', label: 'Max', strokeDasharray: '5 10', strokeWidth: 3}),
    Plot.lineY(phases, {x: "deg", y: "min_adj", stroke: 'black', label: 'Min', strokeDasharray: '5 10', strokeWidth: 3}),
    Plot.gridX({interval: 60, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
});


const phases_difference_plot = Plot.plot({
  y: {domain: [-2.0, 2.0]},
  marks: [
    Plot.lineY(phases, {x: "deg", y: "uv", stroke: 'yellow', label: 'U-V'}),
    Plot.lineY(phases, {x: "deg", y: "vw", stroke: 'cyan', label: 'V-W'}),
    Plot.lineY(phases, {x: "deg", y: "wu", stroke: 'magenta', label: 'W-U'}),
    Plot.gridX({interval: 60, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
});


const phases_rebased_plot = Plot.plot({
  y: {domain: [0, 2.0]},
  marks: [
    Plot.lineY(phases, {x: "deg", y: "adj", stroke: 'black', label: 'Adjustment'}),
    Plot.lineY(phases, {x: "deg", y: "adj_u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: "deg", y: "adj_v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: "deg", y: "adj_w", stroke: 'blue', label: 'W'}),
    Plot.gridX({interval: 60, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
});



// Print the lookup table in chunks of 16 elements per line; indenting each line by a tab character.
const chunk_size = 16;
function chunk_array(arr) {
  return Array.from({length: Math.ceil(arr.length / chunk_size)}, (_, i) => arr.slice(i * chunk_size, i * chunk_size + chunk_size));
}

const phases_waveform = d3.range(angle_base / 4).map((x) => (x * Math.PI / (angle_base / 4))).map(phase_func);

const phases_wave_form_lookup_table = `const uint16_t phases_waveform[${angle_base / 4}] = {\n    ${chunk_array(phases_waveform).map(chunk => chunk.map(d => (d.adj_u / max_adj * pwm_base).toFixed(0).padStart(4, " ")).join(', ')).join(',\n    ')}\n};`;

const sin_waveform = d3.range(angle_base / 4).map((x) => (x * Math.PI / 2 / (angle_base / 4))).map(t => ({t, sin: Math.sin(t)}));

const sin_lookup_table = `const uint16_t sin_lookup[${angle_base / 4}] = {\n    ${chunk_array(sin_waveform).map(chunk => chunk.map(({sin}) => (Math.round(sin * angle_base).toFixed(0).padStart(5, " "))).join(', ')).join(',\n    ')}\n};`;



function funky_atan2(y, x){
  const c = 4; // constant to avoid division by zero

  let result = 0;

  if (x < 0) {
    // Rotate 180 degrees
    result += Math.PI;
    [x, y] = [-x, -y];
  }

  if (y < 0) {
    // Rotate 90 degrees
    result += 3 * Math.PI / 2;
    [x, y] = [-y, x];
  }

  // Now x and y are both positive.

  // Final adjustment; divide the first quadrant into 2 parts by y == x
  // and compute the complementary angle for y > x that we mirror onto the result.
  result += x >= y ? (y / (x + y/c)) : (Math.PI / 2 - x / (y + x/c));

  return (result + Math.PI) % (2 * Math.PI) - Math.PI;
}

const atan_approx = phi.map(t => {
  const deg = t * 180 / Math.PI;
  const m = 180;
  const y = m * Math.sin(t);
  const x = m * Math.cos(t);


  const atan = Math.atan2(y, x) * 180 / Math.PI;
  const atan_approx = funky_atan2(y, x) * 180 / Math.PI;

  return {deg, x, y, atan, atan_approx};
});

const atan_approx_plot = Plot.plot({
  y: {domain: [-180, 180]},
  marks: [
    Plot.lineY(atan_approx, {x: "deg", y: "atan", stroke: 'black', label: 'atan'}),
    Plot.lineY(atan_approx, {x: "deg", y: "atan_approx", stroke: 'red', label: 'atan approx'}),
    Plot.lineY(atan_approx, {x: "deg", y: "x", stroke: 'lightblue', label: 'X'}),
    Plot.lineY(atan_approx, {x: "deg", y: "y", stroke: 'lightgreen', label: 'Y'}),
    Plot.gridX({interval: 60, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 90, stroke: 'black', strokeWidth : 2}),
  ]
});

```