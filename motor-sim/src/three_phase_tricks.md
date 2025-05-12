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

<pre>${phases_wave_form_lookup_table}</pre>
<pre>${sin_lookup_table}</pre>
</main>


```js

import {angle_base, pwm_base} from "./components/motor_constants.js";

const phi = d3.range(angle_base).map(x => 2 * Math.PI * x / angle_base);
const phases = phi.map(t => {
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
  return {sin, deg, u, v, w, adj: -adj, max, min, max_adj, min_adj, adj_u, adj_v, adj_w, uv, vw, wu};
});

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
const phases_chunked = Array.from({length: Math.ceil(phases.length / chunk_size)}, (_, i) => phases.slice(i * chunk_size, i * chunk_size + chunk_size));

const phases_wave_form_lookup_table = `const uint16_t phases_waveform[${angle_base}] = {\n    ${phases_chunked.map(chunk => chunk.map(d => (d.adj_u / max_adj * pwm_base).toFixed(0).padStart(4, " ")).join(', ')).join(',\n    ')}\n};`;

const sin_lookup_table = `const uint16_t sin_lookup[${angle_base}] = {\n    ${phases_chunked.map(chunk => chunk.map(d => (d.sin * angle_base).toFixed(0).padStart(5, " ")).join(', ')).join(',\n    ')}\n};`;