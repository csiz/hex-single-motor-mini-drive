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
```

```js
const truncated_normal_domain = [-90, +90];
const truncated_normal_span = truncated_normal_domain[1] - truncated_normal_domain[0];

const truncated_normal_position_input = Inputs.range(truncated_normal_domain, {
  value: 0,
  step: 0.1,
  label: "Estimated angle:",
});

const truncated_normal_position = Generators.input(truncated_normal_position_input);

const truncated_normal_position_σ_input = Inputs.range([0, truncated_normal_span], {
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

const truncated_normal_mode_functions = {
  "Pause": function *(){ return; },
  "Slide angle": function *(){
    while (true) {
      for (let x of d3.range(0, 1, 1.0/(3*60))) {
        truncated_normal_position_input.value = truncated_normal_domain[0] + truncated_normal_span * x;
        truncated_normal_position_input.dispatchEvent(new Event("input"));
        yield;
      }
    }
  },
  "Loop uncertainty": function *(){
    while (true) {
      for (let x of d3.range(0, 1, 1.0/(3*60))) {
        truncated_normal_position_σ_input.value = 1.0 + truncated_normal_span * (Math.sin(2 * Math.PI * x) + 1.0) / 2.0;
        truncated_normal_position_σ_input.dispatchEvent(new Event("input"));
        yield;
      }
    }
  },
};

const truncated_normal_mode_input = Inputs.radio(Object.keys(truncated_normal_mode_functions), {
  value: "Slide angle",
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
const truncated_normal_animation = truncated_normal_mode_functions[truncated_normal_mode]();
```

```js

const truncated_normal_data = d3.range(...truncated_normal_domain, truncated_normal_span / 500).map((x) => {
  // Using the notation from wikipedia: https://en.wikipedia.org/wiki/Truncated_normal_distribution

  
  const μ = truncated_normal_position;
  const σ = truncated_normal_position_σ;
  const a = truncated_normal_lower;
  const α = (a - μ) / truncated_normal_lower_σ;
  const b = truncated_normal_upper;
  const β = (b - μ) / truncated_normal_upper_σ;

  const pdf_μ = pdf_normal(x, μ, σ) / pdf_normal(0, 0, σ);
  const pdf_a = pdf_normal(a, x, truncated_normal_lower_σ);
  const cdf_a = cdf_normal(a, x, truncated_normal_lower_σ);
  const pdf_b = pdf_normal(x, b, truncated_normal_upper_σ);
  const cdf_b = cdf_normal(x, b, truncated_normal_upper_σ);

  const untriggered_pdf_μ = pdf_μ * (1.0 - cdf_a) * (1.0 - cdf_b);


  return {
    x,
    μ,
    a,
    b,
    pdf_μ,
    pdf_a,
    pdf_b,
    cdf_a,
    cdf_b,
    untriggered_pdf_μ,
  };
});

const normalized_data = normalize_extent(truncated_normal_data, ["pdf_μ", "untriggered_pdf_μ", "pdf_a", "pdf_b"]);

truncated_normal_plot.update({data: normalized_data, truncated_normal_lower, truncated_normal_upper, truncated_normal_position});
```


```js
const truncated_normal_plot = plot_lines({
  subtitle: "Truncated Normal Distribution",
  description: "Example truncating a normal distribution to a lower and upper bound (for example the previous and next hall sector thresholds).",
  width: 1200, height: 300,
  y_domain: [0, 1],
  x: "x",
  x_label: "Distance to center",
  y_label: "Y",
  channels: [
    {
      y: "pdf_μ", label: "PDF μ", color: colors.u,
      draw_extra: setup_v_line({
        x_value: (draw_data) => draw_data.truncated_normal_position,
        dasharray: "2,5",
      }),
    },
    {
      y: "untriggered_pdf_μ", label: "PDF μ (bounded)", color: d3.color(colors.u).darker(1),
      draw_extra: setup_faint_area({
        y0: 0, 
        y1: "untriggered_pdf_μ",
      }),
    },
    {
      y: "pdf_a", label: "PDF a", color: colors.a,
      draw_extra: setup_v_line({
        x_value: (draw_data) => draw_data.truncated_normal_lower,
        dasharray: "2,9",
      }),
    },
    {
      y: "pdf_b", label: "PDF b", color: colors.b,
      draw_extra: setup_v_line({
        x_value: (draw_data) => draw_data.truncated_normal_upper,
        dasharray: "2,7"
      }),
    },
    {y: "cdf_a", label: "CDF a", color: d3.color(colors.a).darker(1)},
    {y: "cdf_b", label: "CDF b", color: d3.color(colors.b).darker(1)},
  ],
});

autosave_inputs({truncated_normal_plot});

```

```js

import {plot_lines, setup_faint_area, horizontal_step, setup_v_line} from "./components/plotting_utils.js";
import {autosave_inputs} from "./components/input_utils.js";
import {cdf_normal, pdf_normal} from "./components/stats_utils.js";

function normalize_extent(data, ys) {
  const norms = ys.map(y => {
    const extent = d3.extent(data, (d) => d[y]);
    const min = extent[0];
    const span = extent[1] - extent[0];
    return {min, span};
  });

  return data.map((d) => {
    const new_d = {...d};
    ys.forEach((y, i) => {
      const norm = norms[i];
      new_d[y] = (d[y] - norm.min) / norm.span;
    });
    return new_d;
  });
}
```
