---
title: Normal distribution tricks
---

<main class="hero">

Kalman Filter with Hall Sensors and EMF Readings
------------------------------------------------

We're going to implement a sneaky Kalman filter from a Bayesian perspective. 

### Inputs

As inputs we have hall sensors and the EMF voltage readings. We might also
have 1 or more magnetic rotary encoders with absolute position (multiple
encoders allow us to measure internal torque on the transmission, if we
know the elastic constant of the transmission mechanism).


* 3 hall sensors with binary state, they are ideally placed 120 degrees apart, but as we shall
measure, they are not that precise. The hall sensors toggle when the magnetic field over the
sensor crosses 0 and switches sign (with some hysterisis). The result is that we have 6 valid
regions for the motor position. The crossover point between each region is known, and it is
different if moving clockwise or counter-clockwise. The rotor also tends to rest within each
of these sectors, however note that the resting position is aligned with the stator iron cores 
not the hall sensors. We can notice this effect in the oscilations of the EMF voltage magnitude.

* EMF voltage readings, which are a noisy measurement of the rotor position. The EMF angle is
unknown below a certain speed where the EMF voltage readout is near the least significant bit
(LSB) of the ADC and therefore quite noisy. The threshold speed is surprisingly low, about 2%
of the maximum speed. Above this speed we can use the EMF voltage to estimate the rotor position.

* A magnetic rotary encoder with absolute position (like AS5600 with 10bit resolution).
This is a very precise measurement of the rotor position, but it is not fast enough for
the current control loop. However we can use it for absolute position tracking of the
output, after gearing. It will allow us to measure backlash of the gears and possibly
internal torque on the transmission if compared with the EMF voltage readings.


### Assumptions / Probabilistic setup

We assume all our reference angles are gaussian distributed, with a known/calibrated mean and standard deviation.

1. The motor is stationary at the start.
2. The initial position is the center of the current hall sector.
3. We know/measure the hall sector crossover points and variances for both rotation directions.
4. We can measure the EMF angle and its variance using an exponential moving average (EMA).

### Model

To model the a crossover transition between hall sectors, we will assume the motor position is
sampled from a prior gaussian distribution with the mean and variance predicted in the Kalman step.
The hall crossover point will also be sampled from the calibrated gaussian distribution of the hall 
sector crossover points. The transition should occur when the sampled motor position is greater than 
the sampled hall crossover point. Then we measure the whether the transition actually occurred and
calculate the posterior distribution of the motor position. We'll use the posterior distribution
as the measurement input to the Kalman update. Above the threshold speed, we can incorporate the EMF
angle and its variance as a secondary input.



Bayesian Inference
------------------

There are some challenges implementing the Kalman filter given the high frequency of the EMF readings
and the low frequency and binary nature of the hall sensor readings. To account for the low frequency
hall sensor we could set the confidence to 0 when no hall transition is detected. But we can do better!
The solution will also solve our second problem, to slowly bring the position estimate back to the center
of the current hall sector when no hall transition is detected. What we'll do is to predict how likely
it is that we should have detected a hall transition given our position statistics. If we should have
detected a transition but didn't we will adjust the position estimate towards the center of the current 
hall sector. This is an approximation to the actual posterior distribution of the position statistics.
We can visualize the actual posterior distribution in the plot below because we have enough computing 
power on any device that can run a browser.

We consider the probability of the motor position being greater than the hall sector crossover point, given the
current position estimate and its variance. Google Gemini has blessed us with a clear and accurate description:

It sounds like the project involves a **Bayesian inference problem where our data (or evidence) comes from a probabilistic comparison.**

* We have a **parameter of interest**, let's call it ${tex`X_0`}. Your **prior belief** about ${tex`X_0`} is described by a Gaussian probability density function (PDF) with mean ${tex`\mu_0`} and standard deviation ${tex`\sigma_0`}, which we can denote as ${tex`p(x_0) = \mathcal{N}(x_0|\mu_0, \sigma_0^2)`}.

* We don't observe ${tex`X_0`} directly. Instead, you perform an experiment or make an observation that involves comparing ${tex`X_0`} to another quantity, ${tex`X_1`}. This ${tex`X_1`} is also a random variable, drawn from a Gaussian distribution with mean ${tex`\mu_1`} and standard deviation ${tex`\sigma_1`}.

* The **"measurement"** or **data** we obtain is the outcome of this comparison:
    1.  Either ${tex`X_0 > X_1`} (a sample from our prior is greater than a sample from the second Gaussian).
    2.  Or ${tex`X_0 < X_1`} (a sample from our prior is less than a sample from the second Gaussian).

* We are constructing the **likelihood function** for these two possible outcomes, given a specific value of ${tex`X_0=x_0`}:
    * The likelihood of observing "${tex`X_0 > X_1`}" given ${tex`X_0=x_0`} is ${tex`P(X_1 < x_0 | X_0=x_0)`}. This is precisely the cumulative distribution function (CDF) of ${tex`X_1`} evaluated at ${tex`x_0`}, let's call this ${tex`\text{CDF}_{X_1}(x_0)`}.
    * The likelihood of observing "${tex`X_0 < X_1`}" given ${tex`X_0=x_0`} is ${tex`P(X_1 > x_0 | X_0=x_0)`}. This is ${tex`1 - \text{CDF}_{X_1}(x_0)`}.

* We are then using Bayes' theorem to find the **posterior probability distribution** for ${tex`X_0`} under each of these two data scenarios:
    1.  If our measurement is "${tex`X_0 > X_1`}":
        ${tex`p(x_0 | X_0 > X_1) \propto p(x_0) \cdot \text{CDF}_{X_1}(x_0)`}
    2.  If our measurement is "${tex`X_0 < X_1`}":
        ${tex`p(x_0 | X_0 < X_1) \propto p(x_0) \cdot (1 - \text{CDF}_{X_1}(x_0))`}

The "problem" can be described using terms like:

* **Bayesian inference with a probabilistic comparative likelihood:** The likelihood is not based on a direct measurement of ${tex`X_0`} with some noise, but on the probability of ${tex`X_0`} satisfying an inequality with respect to another random variable.
* **Updating with binary outcome from stochastic comparison:** Our data is binary (greater than / less than), and this outcome depends on a comparison where one of the terms (${tex`X_1`}) is stochastic.
* **A model with a Probit-type likelihood:** The use of a Gaussian CDF as the likelihood function is characteristic of Probit models, which are often used for binary outcomes that depend on an underlying latent variable exceeding a threshold. Here, ${tex`X_0`} is like the latent variable, and ${tex`X_1`} is a stochastic threshold.

In essence, we're updating our beliefs about ${tex`X_0`} based on whether it "passed" or "failed" a test where the benchmark (${tex`X_1`}) was itself uncertain. The two posterior distributions represent our updated beliefs about ${tex`X_0`} in these two distinct scenarios.

### The absence of evidence is evidence of the absence

Enough with the maths, the plot below shows how we should update our belief about the motor position every motor cycle. First we predict the our prior for the motor position then we check if a hall transition was detected. 
* If the detection matches our prediction, we gain little information, but confirm our prior is correct, thus the new estimate remains the same as the initial prediction.
* When the prediction is wrong, we update the position for 2 cases:
  1. If the hall transition was detected when we didn't predict it, the motor must have sped up towards the detected sector. The new position is close to the trigger point (the position between 2 sectors).
  2. A transition was predicted but none was detected, then the motor must have slowed down. The updated position moves slightly towards the center of the current sector.

<div class="card tight">
    <div>${bounded_example_input}</div>
    <div>${bounded_example_plot}</div>
</div>


Combining Normal Distributions
------------------------------

Back to math, now that we have a corrected estimate of the motor position we can update our prior belief to incorporate as much information that we have from our measurement (or lack of). According to Bayes' rule the likelihood of the real position is proportional to our prior PDF multiplied by the measurment PDF. There is a special case when both PDFs are Gaussian, the result is also a Gaussian distribution. This special case is the Kalman filter update step derived from first principles. The mathematics of the Kalman gain work out to the same value as the weighting factor we use to combine the two PDFs. The mean and variance of the combined reading are vizualized below [1]:

```tex
\begin{align*}
\mu_{combined} &= \frac{\mu_1 \sigma_2^2 + \mu_2 \sigma_1^2}{\sigma_1^2 + \sigma_2^2} \\
\sigma_{combined}^2 &= \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}
\end{align*}
```

We also consider the case where the confidence in each source is weighted by a factor ${tex`w`}. The
mean can be calculated by iteratively applying to above formula formula for ${tex`w_1`} times the
first source and ${tex`w_2`} times the second source. The variance however would shrink more than
desired due to increased confidence in multiple measurements. To adjust this, we need to scale the
weights to the interval [0, 1], we'll interpret this as confidence in the measurement. The mean 
and variance of the combined reading can then be calculated as follows:

```tex
\begin{align*}
\mu_{combined} &= \frac{w_1 \mu_1 \sigma_2^2 + w_2 \mu_2 \sigma_1^2}{w_1 \sigma_2^2 + w_2 \sigma_1^2} \\
\sigma_{combined}^2 &= \frac{\sigma_1^2 \sigma_2^2}{w_1 \sigma_2^2 + w_2 \sigma_1^2}
\end{align*}
```

<div class="card tight">
    <div>${combined_example_input}</div>
    <div>${combined_example_plot}</div>
</div>


References
----------

* [1] [Products and Convolutions of Gaussian Probability Density Functions P.A. Bromiley](http://www.lucamartino.altervista.org/2003-003.pdf)
* [2] [Kalman Filter - Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
* [3] [Bayesian Inference - Wikipedia](https://en.wikipedia.org/wiki/Bayesian_inference)
* [4] [Probability distribution function - Wikipedia](https://en.wikipedia.org/wiki/Probability_distribution)
* [5] [Normal distribution - Wikipedia](https://en.wikipedia.org/wiki/Normal_distribution)

</main>


```js

const colors = {
  a: "rgb(217, 95, 2)",
  b: "rgb(231, 41, 138)",
  c: "rgb(102, 102, 102)",
  d: "rgb(197, 152, 67)",
  e: "rgb(102, 166, 30)",
  f: "rgb(27, 158, 119)",
  g: "rgb(199, 0, 57)",
  h: "rgb(26, 82, 118)",
  i: "rgb(117, 112, 179)",
};
```

```js
const domain = [-90, +90];
const span = domain[1] - domain[0];

const position_input = inputs_wide_range(domain, {
  value: 0,
  step: 0.1,
  label: "Prior angle:",
});

const position = Generators.input(position_input);

const position_stdev_input = inputs_wide_range([0, span], {
  value: 15,
  step: 0.1,
  label: "Prior angle stdev:",
});

const position_stdev = Generators.input(position_stdev_input);

const upper_input = inputs_wide_range([0, domain[1]], {
  value: 35,
  step: 0.1,
  label: "Hall trigger b",
});
const upper = Generators.input(upper_input);

const upper_stdev_input = inputs_wide_range([0, span / 10], {
  value: 3,
  step: 0.1,
  label: "Hall trigger b stdev:",
});
const upper_stdev = Generators.input(upper_stdev_input);

const upper_weight_input = inputs_wide_range([0, 1], {
  value: 1.0,
  step: 0.01,
  label: "Confidence b should be triggered:",
});

const upper_weight = Generators.input(upper_weight_input);

const lower_input = inputs_wide_range([domain[0], 0], {
  value: -35,
  step: 0.1,
  label: "Hall trigger a:",
});

const lower = Generators.input(lower_input);

const lower_stdev_input = inputs_wide_range([0, span / 10], {
  value: 3,
  step: 0.1,
  label: "Hall trigger a stdev:",
});

const lower_stdev = Generators.input(lower_stdev_input);

const lower_weight_input = inputs_wide_range([0, 1], {
  value: 1.0,
  step: 0.01,
  label: "Confidence a should be triggered:",
});
const lower_weight = Generators.input(lower_weight_input);

const mode_functions = {
  "Pause": function *(){ return; },
  "Slide angle": function *(){
    while (true) {
      for (let x of d3.range(0, 1, 1.0/(3*60))) {
        position_input.value = domain[0] + span * x;
        position_input.dispatchEvent(new Event("input"));
        yield;
      }
    }
  },
  "Loop uncertainty": function *(){
    while (true) {
      for (let x of d3.range(0, 1, 1.0/(3*60))) {
        position_stdev_input.value = 1.0 + span * (Math.sin(2 * Math.PI * x) + 1.0) / 2.0;
        position_stdev_input.dispatchEvent(new Event("input"));
        yield;
      }
    }
  },
};

const mode_input = Inputs.radio(Object.keys(mode_functions), {
  value: "Slide angle",
  label: "Play mode:",
});

const mode = Generators.input(mode_input);

const bounded_example_input = [
  mode_input,
  position_input,
  position_stdev_input,
];

const combined_example_input = [
  lower_input,
  lower_stdev_input,
  lower_weight_input,
  upper_input,
  upper_stdev_input,
  upper_weight_input,
];

```

```js
const animation = mode_functions[mode]();
```

```js

const {mean: ab_mean, stdev: ab_stdev} = weighted_product_of_normals({
  mean_a: lower,
  stdev_a: lower_stdev,
  mean_b: upper,
  stdev_b: upper_stdev,
  weight_a: lower_weight,
  weight_b: upper_weight,
});

const data = d3.range(...domain, span / 500).map((x) => {


  const pdf_prior = pdf_normal(x, position, position_stdev);
  const pdf_a = pdf_normal(x, lower, lower_stdev);
  const cdf_a = cdf_normal(-x, -lower, lower_stdev);
  const pdf_b = pdf_normal(x, upper, upper_stdev);
  const cdf_b = cdf_normal(x, upper, upper_stdev);

  const pdf_a_mul_pdf_b = pdf_a * pdf_b;
  const pdf_ab = pdf_normal(x, ab_mean, ab_stdev);
  

  const untriggered_pdf = pdf_prior * (1.0 - cdf_a) * (1.0 - cdf_b);
  const a_triggered_pdf = pdf_prior * cdf_a;
  const b_triggered_pdf = pdf_prior * cdf_b;


  return {
    x,
    pdf_prior,
    pdf_a_mul_pdf_b,
    pdf_ab,
    pdf_a,
    pdf_b,
    cdf_a,
    cdf_b,
    untriggered_pdf,
    a_triggered_pdf,
    b_triggered_pdf,
  };
});

const normalized_data = normalize_extent(data, [
  "pdf_prior", "untriggered_pdf", "a_triggered_pdf", "b_triggered_pdf", 
  "pdf_a", "pdf_b", "pdf_ab", "pdf_a_mul_pdf_b",
]);

[bounded_example_plot, combined_example_plot].forEach(plot => plot.update({data: normalized_data, lower, upper, position, ab_mean}));
```


```js

const bounded_example_plot = plot_lines({
  subtitle: "Truncated Normal Distribution",
  description: "Example truncating a normal distribution to a lower and upper bound (for example the previous and next hall sector thresholds).",
  width: 1200, height: 300,
  x: "x",
  x_label: "Distance to center",
  x_domain: [-90, +90],
  y_label: "Y (normalized)",
  y_domain: [0, 1],
  channels: [
    {
      y: "pdf_prior", label: "PDF prior", color: colors.i,
      draw_extra: setup_v_line({x_value: (d) => d.position, dasharray: "2,5"}),
    },
    {
      y: "untriggered_pdf", label: "PDF no transition", color: d3.color(colors.i).darker(1),
      draw_extra: setup_faint_area({y0: 0, y1: "untriggered_pdf"}),
    },
    {
      y: "a_triggered_pdf", label: "PDF transition to a", color: colors.h,
      draw_extra: setup_faint_area({y0: 0, y1: "a_triggered_pdf"}),
    },
    {
      y: "b_triggered_pdf", label: "PDF transition to b", color: colors.g,
      draw_extra: setup_faint_area({y0: 0, y1: "b_triggered_pdf"}),
    },
    {
      y: "cdf_a", label: "CDF a", color: d3.color(colors.a).darker(1),
      draw_extra: setup_v_line({x_value: (d) => d.lower, dasharray: "2,9"}),
    },
    {
      y: "cdf_b", label: "CDF b", color: d3.color(colors.b).darker(1),
      draw_extra: setup_v_line({x_value: (d) => d.upper, dasharray: "2,7"}),
    },
  ],
});

const combined_example_plot = plot_lines({
  subtitle: "Combining two normal distributions",
  description: "Example combining two normal distributions with different means and standard deviations.",
  width: 1200, height: 300,
  x: "x",
  x_label: "Distance to center",
  x_domain: [-90, +90],
  y_label: "Y (normalized)",
  y_domain: [0, 1],
  channels: [
    {
      y: "pdf_ab", label: "PDF ab weighted", color: colors.i,
      draw_extra: setup_v_line({x_value: (d) => d.ab_mean, dasharray: "2,5"}),
    },
    {
      y: "pdf_a_mul_pdf_b", label: "PDF a * PDF b", color: colors.c,
      draw_extra: setup_faint_area({y0: 0, y1: "pdf_a_mul_pdf_b"}),
    },
    {
      y: "pdf_a", label: "PDF a", color: colors.a,
      draw_extra: setup_v_line({x_value: (d) => d.lower, dasharray: "2,9"}),
    },
    {
      y: "pdf_b", label: "PDF b", color: colors.b,
      draw_extra: setup_v_line({x_value: (d) => d.upper, dasharray: "2,7"}),
    },
  ],
});



autosave_inputs({
  bounded_example_plot,
  combined_example_plot,
});
```



```js

import {plot_lines, setup_faint_area, horizontal_step, setup_v_line} from "./components/plotting_utils.js";
import {autosave_inputs, inputs_wide_range} from "./components/input_utils.js";
import {cdf_normal, pdf_normal, quantile_normal, weighted_product_of_normals} from "./components/stats_utils.js";

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

