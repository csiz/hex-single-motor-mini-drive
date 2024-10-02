import {html} from "htl";
import {require} from "d3-require";
import {md as md_loader} from "npm:@observablehq/stdlib/src/md.js";
import * as Plot from "npm:@observablehq/plot";

export const md = await md_loader(require);

export function note (text) {
	return html`<div class="tooltip">*<span class="tooltiptext">${text}</span></div>`;
}

export function link (url) {
	return html`<a href="${url}" target="_blank">${url}</a>`;
}

const default_sparkline_plot = {label: "<sparkline>", x: "t", y: null, stroke: "black", fill: "none"};
const default_sparkline_options = {domain: null, height: 60};

export function sparkline(data, plots=default_sparkline_plot, options=default_sparkline_options){
  const {domain, height} = Object.assign({}, default_sparkline_options, options);
  
  if (!Array.isArray(plots)) plots = [plots];
  
  plots.map((plot) => Object.assign({}, default_sparkline_plot, plot));

  const plot = Plot.plot({
    height,
    axis: null,
    y: domain == null ? undefined : {domain},
    marks:[
      plots.map(({x, y, stroke, fill}) => Plot.lineY(data, {x, y, stroke, fill})),
      Plot.ruleY([0], {stroke: "gray", strokeDasharray: "8,2"}),
    ],
  });

  return html`
    <div class="card tight" style="display: flex; align-items: center;">
      <div style="display: flex; flex-direction: column;">
        ${plots.map(({label, stroke}) => html`<label style="min-width: 120px; margin-right: 6.5px; color: ${stroke};">${label}</label>`)}
      </div>
      <div>${plot}</div>
    </div>
  `;
}
