import * as Plot from "@observablehq/plot";
import {html} from "htl";
import {min, max} from "d3-array";

const default_sparkline_plot = {label: "<sparkline>", x: "t", y: null, stroke: "black", fill: "none"};
const default_sparkline_options = {least_domain: null, height: 60};

export function sparkline(data, plots=default_sparkline_plot, options=default_sparkline_options){
  const {least_domain, height} = Object.assign({}, default_sparkline_options, options);
  
  if (!Array.isArray(plots)) plots = [plots];
  
  plots.map((plot) => Object.assign({}, default_sparkline_plot, plot));

  const min_domain = min(data.map((item) => min(plots.map(({y}) => item[y]))));
  const max_domain = max(data.map((item) => max(plots.map(({y}) => item[y]))));

  const domain = least_domain ? [
    Math.min(min_domain, -max_domain, least_domain[0]),
    Math.max(max_domain, -min_domain, least_domain[1]),
  ] : [
    Math.min(min_domain, -max_domain),
    Math.max(max_domain, -min_domain),
  ];

  const plot = Plot.plot({
    height,
    axis: null,
    y: {domain},
    marks:[
      plots.map(({x, y, stroke, fill}) => Plot.lineY(data, {x, y, stroke, fill})),
      Plot.ruleY([0], {stroke: "gray", strokeDasharray: "8,2"}),
    ],
  });

  return html`<div class="card tight" style="display: flex; align-items: center;">
    <div style="display: flex; flex-direction: column;">
      ${plots.map(({label, stroke}) => html`<label style="min-width: 120px; margin-right: 6.5px; color: ${stroke};">${label}</label>`)}
    </div>
    <div>${plot}</div>
  </div>`;
}