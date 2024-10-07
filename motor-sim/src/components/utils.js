import {html} from "htl";

import {Generators} from "observablehq:stdlib";
import _ from "lodash";

export function note (text) {
	return html`<div class="tooltip">*<span class="tooltiptext">${text}</span></div>`;
}

export function link (url) {
	return html`<a href="${url}" target="_blank">${url}</a>`;
}

export class TimingStats {
  constructor() {
    this.last_time = Date.now();
    this.fps = 0.0;
    this.gamma = 0.9;
  }

  update() {
    const now = Date.now();
    const wall_dt = now - this.last_time;
    this.last_time = now;
    this.fps = this.gamma * this.fps + (1.0-this.gamma) * (1000.0 / wall_dt);
  };
}


export function ThrottledMutable(delay, value) {
  let change = (v) => {value = v};
  
  return Object.defineProperty(
    Generators.observe((notify) => {
      change = notify;
      if (value !== undefined) notify(value);
    }),
    "value",
    {
      get: () => value,
      set: _.throttle((x) => {change((value = x)); return x;}, delay),
    }
  );
}


