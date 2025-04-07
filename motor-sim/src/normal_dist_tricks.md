---
title: Normal Distribution Tricks
---

<main class="hero">

Bounded Normal Distribution
---------------------------

As the rotor spins it will trigger the hall sensors allowing us to estimate the rotor position. However, the hall sensors
are not perfect and will trigger at random angles. Our position estimate will also be uncertain between the hall sensor 
trigger events. But is the absence of evidence, evidence itself? Well, maybe... We can combine our position estimate with
the probability of the next hall sensor trigger to get a better estimate of the rotor position. Whilst our position estimate
is between the neighbouring hall sensor triggers we wouldn't expect a trigger, so we don't need to correct our estimate. When
we move past a hall sensor we should expect a trigger, if that does not happen then we must have slowed down so we should nudge
our estimate back. Finally, as our uncertainty increases with time we find ourselves equally likely to trigger the hall sensor
on either side, so our estimate should be pulled back to the centre between the hall sensors.


<div class="card tight">
    <div>${truncated_normal_input}</div>
    <div>${truncated_normal_plot}</div>
</div>

</main>


```js

const colors = {
  u: "rgb(117, 112, 179)",
  a: "rgb(217, 95, 2)",
  b: "rgb(231, 41, 138)",
  c: "rgb(102, 102, 102)",
  d: "rgb(197, 152, 67)",
  e: "rgb(102, 166, 30)",
  f: "rgb(27, 158, 119)",
  g: "rgb(199, 0, 57)",
  h: "rgb(26, 82, 118)",
};

const truncated_normal_domain = [-90, +90];
const truncated_normal_span = truncated_normal_domain[1] - truncated_normal_domain[0];

const truncated_normal_position_input = Inputs.range(truncated_normal_domain, {
  value: 0,
  step: 0.1,
  label: "Estimated angle:",
});

const truncated_normal_position = Generators.input(truncated_normal_position_input);

const truncated_normal_position_σ_input = Inputs.range([0, truncated_normal_span / 5], {
  value: 15,
  step: 0.1,
  label: "Estimated angle σ:",
});

const truncated_normal_position_σ = Generators.input(truncated_normal_position_σ_input);

const truncated_normal_upper_input = Inputs.range([0, truncated_normal_domain[1]], {
  value: 35,
  step: 0.1,
  label: "Truncated normal next hall trigger:",
});
const truncated_normal_upper = Generators.input(truncated_normal_upper_input);

const truncated_normal_upper_σ_input = Inputs.range([0, truncated_normal_span / 10], {
  value: 3,
  step: 0.1,
  label: "Truncated normal next hall trigger σ:",
});
const truncated_normal_upper_σ = Generators.input(truncated_normal_upper_σ_input);

const truncated_normal_lower_input = Inputs.range([truncated_normal_domain[0], 0], {
  value: -35,
  step: 0.1,
  label: "Truncated normal previous hall trigger:",
});

const truncated_normal_lower = Generators.input(truncated_normal_lower_input);

const truncated_normal_lower_σ_input = Inputs.range([0, truncated_normal_span / 10], {
  value: 3,
  step: 0.1,
  label: "Truncated normal previous hall trigger σ:",
});

const truncated_normal_lower_σ = Generators.input(truncated_normal_lower_σ_input);

const truncated_normal_mode_input = Inputs.radio(["User selection", "Slide estimated angle", "Loop estimation uncertainty"], {
  value: "Slide estimated angle",
  label: "Play mode:",
});

const truncated_normal_mode = Generators.input(truncated_normal_mode_input);

const truncated_normal_input = [
  truncated_normal_mode_input,
  truncated_normal_position_input,
  truncated_normal_position_σ_input,
  truncated_normal_upper_input,
  truncated_normal_upper_σ_input,
  truncated_normal_lower_input,
  truncated_normal_lower_σ_input,
];

```

```js
const truncated_normal_shown_position = 
  truncated_normal_mode == "User selection" ? truncated_normal_position :
  truncated_normal_mode == "Slide estimated angle" ? truncated_normal_domain[0] + truncated_normal_span * (now / 10000.0 % 1.0) :
  truncated_normal_mode == "Loop estimation uncertainty" ? truncated_normal_upper - 2 * truncated_normal_upper_σ :
  0.0;

const truncated_normal_shown_position_σ = 
  truncated_normal_mode == "User selection" ? truncated_normal_position_σ :
  truncated_normal_mode == "Slide estimated angle" ? truncated_normal_position_σ :
  truncated_normal_mode == "Loop estimation uncertainty" ? 1.0 + truncated_normal_span * (Math.sin(2 * Math.PI * (now / 10000.0 % 1.0)) + 1.0) / 2.0:
  0.0;

```

```js
const truncated_normal_data = d3.range(...truncated_normal_domain, truncated_normal_span / 500).map((x) => {
  const μ = truncated_normal_shown_position;
  const σ = truncated_normal_shown_position_σ;
  const a = truncated_normal_lower;
  const α = (a - μ) / truncated_normal_lower_σ;
  const cdf_α = cdf_normal(α, 0.0, 1.0);
  const pdf_α = pdf_normal(α, 0.0, 1.0);
  const b = truncated_normal_upper;
  const β = (b - μ) / truncated_normal_upper_σ;
  const cdf_β = cdf_normal(β, 0.0, 1.0);
  const pdf_β = pdf_normal(β, 0.0, 1.0);

  const pdf_μ = pdf_normal(x, μ, σ) / pdf_normal(0, 0, σ);
  const pdf_a = pdf_normal(a, x, truncated_normal_lower_σ);
  const cdf_a = cdf_normal(a, x, truncated_normal_lower_σ);
  const pdf_b = pdf_normal(x, b, truncated_normal_upper_σ);
  const cdf_b = cdf_normal(x, b, truncated_normal_upper_σ);

  const untriggered_pdf_μ = pdf_μ * (1.0 - cdf_a) * (1.0 - cdf_b); 

  return {
    x, 
    pdf_μ,
    pdf_a,
    pdf_b,
    cdf_a,
    cdf_b,
    untriggered_pdf_μ,

    y: -pdf_β/cdf_β, 
    cdf_β, 
    pdf_β,
  };
});

const truncated_normal_plot = plot_multiline({
  data: truncated_normal_data,
  store_id: "truncated_normal_plot",
  selection: null,
  subtitle: "Truncated Normal Distribution",
  description: "Example truncating a normal distribution to a lower and upper bound (the next hall sector thresholds).",
  width: 1200, height: 300,
  x_options: {},
  y_options: {domain: [0, 1]},
  x: "x",
  y: "y",
  x_label: "Distance to center",
  y_label: "Y",
  channel_label: "Phase",
  channels: [
    {y: "pdf_μ", label: "PDF μ", color: colors.u},
    {y: "untriggered_pdf_μ", label: "PDF μ (bounded)", color: d3.color(colors.u).darker(1), area_y: "untriggered_pdf_μ"},
    {y: "pdf_a", label: "PDF a", color: colors.a},
    {y: "pdf_b", label: "PDF b", color: colors.b},
    {y: "cdf_a", label: "CDF a", color: d3.color(colors.a).darker(1)},
    {y: "cdf_b", label: "CDF b", color: d3.color(colors.b).darker(1)},
  ],
  mark_function: (selected_data, options) => Plot.lineY(selected_data, Plot.normalizeY("extent", options)),
  grid_marks: [
    Plot.gridX({stroke: 'black', strokeWidth : 1}),
    Plot.gridY({stroke: 'black', strokeWidth : 1}),
  ],
  other_marks: [
    (selected_data, options) => Plot.areaY(selected_data, Plot.normalizeY("extent", {...options, y: "area_y", fill: options.z, opacity: 0.2})),
    Plot.ruleX([truncated_normal_lower], {stroke: colors.a, strokeWidth: 2, strokeDasharray: "2,5"}),
    Plot.ruleX([truncated_normal_upper], {stroke: colors.b, strokeWidth: 2, strokeDasharray: "2,5"}),
    Plot.ruleX([truncated_normal_shown_position], {stroke: colors.u, strokeWidth: 2, strokeDasharray: "2,5"}),
  ],
});

```

```js

import {plot_multiline, horizontal_step} from "./components/plotting_utils.js";
import {cdf_normal, pdf_normal} from "./components/stats_utils.js";
```
