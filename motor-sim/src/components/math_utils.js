import * as d3 from "d3";

/* Get `n` evenly spaced values from 0 to `max_value` inclusive. */
export function even_spacing(max_value, n) {
    return d3.range(n).map((i) => i / (n - 1) * max_value);
  }
  
  /* Get an interpolator for a piecewise linear function specified by the points `X` and `Y`. */
  export function piecewise_linear({X, Y}) {
    const n = X.length;
    const m = Y.length;
    if (n !== m) throw new Error("X and Y must have the same length");
    if (n < 2) throw new Error("X and Y must have at least 2 points");
  
    // Check if X is sorted
    for (let i = 1; i < n; i++) {
      if (X[i] < X[i - 1]) throw new Error("X must be sorted");
    }
  
    const slopes = new Array(n - 1);
    const intercepts = new Array(n - 1);
    for (let i = 0; i < n - 1; i++) {
      slopes[i] = (Y[i + 1] - Y[i]) / (X[i + 1] - X[i]);
      intercepts[i] = Y[i] - slopes[i] * X[i];
    }
  
    return function(x) {
      let i = (x <= X[0]) ? 0 : (x > X[n - 1]) ? n - 2 : d3.bisectRight(X, x, 1, n - 1) - 1;
      return slopes[i] * x + intercepts[i];
    };
  }
  
  /* Get an interpolator for a piecewise linear function specified by the points `Y` and a range from `x_min` to `x_max` inclusive. */
  export function even_piecewise_linear({x_min, x_max, Y}) {
    const n = Y.length;
    if (n < 2) throw new Error("Y must have at least 2 points");
  
    const x_period = (x_max - x_min) / (n - 1);
  
    const slopes = new Array(n - 1);
    const intercepts = new Array(n - 1);
    for (let i = 0; i < n - 1; i++) {
      slopes[i] = (Y[i + 1] - Y[i]) / x_period
      intercepts[i] = Y[i] - slopes[i] * (x_period * i + x_min);
    }
  
    return function(x) {
      let i = (x <= x_min) ? 0 : (x > x_max) ? n - 2 : Math.ceil((x - x_min) / x_period) - 1;
      return slopes[i] * x + intercepts[i];
    };
  }
  