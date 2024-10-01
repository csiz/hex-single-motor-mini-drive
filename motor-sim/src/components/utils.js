import {html} from "npm:htl";
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

export function sparkline(data, {label = "<sparkline>", x = "t", y = null, domain = null, height = 60, stroke = "black", fill = "none"}){
  const plot = Plot.plot({
    height,
    axis: null,
    y: domain == null ? undefined : {domain},
    marks:[
      Plot.lineY(data, {x, y, stroke, fill}),
      Plot.ruleY([0], {stroke: "gray", strokeDasharray: "8,2"}),
    ],
  });

  return html`
    <div class="card tight" style="display: flex; align-items: center;">
      <label style="min-width: 120px; margin-right: 6.5px;">${label}</label>
      <div>${plot}</div>
    </div>
  `;
}
