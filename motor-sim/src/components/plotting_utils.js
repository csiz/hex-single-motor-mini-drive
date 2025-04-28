import _ from "lodash";
import {html} from "htl";
import {Mutable} from "observablehq:stdlib";
import * as Plot from "@observablehq/plot";
import * as Inputs from "@observablehq/inputs";
import * as d3 from "d3";

import {set_stored, get_stored_or_default} from "./local_storage.js";

export function merge_input_properties(input, value) {
  input.value = {...input.value, ...value};
  input.dispatchEvent(new Event("input", {bubbles: true}));
}

export function replace_element(element, new_element) {
  element.replaceWith(new_element);
  return new_element;
}

export function update_multiline_data(multiline_plot, value) {
  multiline_plot.input.value = {...multiline_plot.input.value, ...value};
  multiline_plot.input.dispatchEvent(new Event("plot-data"));
}

export function autosave_multiline_inputs(multiline_plots, delay_millis = 100) {
    Object.entries(multiline_plots).forEach(([key, multiline_plot]) => {
      const input = multiline_plot.input;
      // Merge the stored values with the current input values; preserving data.
      merge_input_properties(input, get_stored_or_default(key, {}));

      // Add an event listener to save the input values to local storage.
      input.addEventListener("input", _.debounce(() => {
        // Ignore plotting data so we don't store too much data in local storage.
        const {show, shown_marks} = input.value;
        set_stored(key, {show, shown_marks});
      }, delay_millis));
    });
}

function expand_z_channels({data, x_label, y_label, x, channels}){

  return data.flatMap((d, i, data) => {
    return channels.map(({y, label, color, ...other_y}) => {
      return Object.fromEntries([
        ...Object.entries(other_y).map(([key, value]) => [key, _.isFunction(value) ? value(d, i, data) : d[value]]),
        [x_label, _.isFunction(x) ? x(d, i, data) : d[x]],
        [y_label, _.isFunction(y) ? y(d, i, data) : d[y]],
        ["z", label],
        ["color", color],
      ]);
    });
  });
}


export function plot_multiline(options){
  const {
    data = [], 
    x_options = {}, 
    y_options = {},
    width, height,
    x_label, y_label,
    subtitle, description,
    x, 
    channels,
    grid_marks = [],
    other_marks = [],
    curve = undefined,
    default_shown_marks = undefined,
    default_show = true,
  } = options;
  

  const description_element = html`<p>${description}</p>`;

  const checkbox_info = [...channels, {label: "Grid", color: "grey"}];

  let input = Inputs.input({
    show: default_show,
    shown_marks: default_shown_marks ?? checkbox_info.map(({label}) => label),
    data,
    x_options, 
    y_options,
  });

  // First, make the title into a checkbox to toggle the plot on and off.
  const subtitle_checkbox = Inputs.checkbox([subtitle], {
    value: input.value.show ? [subtitle] : [],
    format: (subtitle) => html`<h4 style="min-width: 20em; font-size: 1.5em; font-weight: normal;">${subtitle}</h4>`,
  });
  
  // Then, make the marks into checkboxes to toggle them on and off.
  const marks_checkboxes = Inputs.checkbox(
    checkbox_info.map(({label}) => label), 
    {
      value: input.value.shown_marks,
      label: "Display:",
      format: (label, i) => html`<span style="border-bottom: solid 3px ${checkbox_info[i].color}; margin-bottom: -3px;">${label}</span>`,
    },
  );
  


  function create_plot_element({shown_marks, data, x_options, y_options}) {
    if (data.length === 0) return html`<div>Waiting for data.</div>`;

    const selected_channels = channels.filter(({label}) => shown_marks.includes(label));
    const selected_data = expand_z_channels({data, x_label, y_label, x, channels: selected_channels});

    return Plot.plot({
      width, height,
      x: {label: x_label, ...x_options},
      y: {label: y_label, domain: selected_data.length > 0 ? undefined : [0, 1], ...y_options},
      color: {
        // legend: true,
        domain: channels.map(({label}) => label),
        range: channels.map(({color}) => color),
      },
      marks: [
        Plot.lineY(selected_data, {x: x_label, y: y_label, stroke: "z", z: "z", curve}),
        Plot.crosshairX(selected_data, {x: x_label, y: y_label, color: "z", ruleStrokeWidth: 3}),
        Plot.dot(selected_data, Plot.pointerX({x: x_label, y: y_label, stroke: "z"})),
        Plot.text(
          selected_data,
          Plot.pointerX({
            px: x_label, py: y_label, fill: "z",
            dy: -17, frameAnchor: "top-right", monospace: true, fontSize: 14, fontWeight: "bold",
            text: (d) => `${x_label}: ${d[x_label]?.toFixed(3).padStart(9)} | ${y_label}: ${d[y_label]?.toFixed(3).padStart(9)} | ${d["z"].padStart(20)}`,
          }),
        ),
        ...(marks_checkboxes.value.includes("Grid") ? grid_marks : []),
        ...other_marks.map(mark => _.isFunction(mark) ? mark(selected_data, {x: x_label, y: y_label, z: "z"}, input.value) : mark),
      ],
    });
  }


  let plot_element = create_plot_element(input.value);

  const plot_details = html`<div>${description_element}${marks_checkboxes}${plot_element}</div>`;

  let element = html`<figure>${subtitle_checkbox}${plot_details}</figure>`;

  element.input = input;

  subtitle_checkbox.addEventListener("input", function(){
    const show = subtitle_checkbox.value.length > 0;
    merge_input_properties(input, {show});
  });

  marks_checkboxes.addEventListener("input", function(){
    const shown_marks = marks_checkboxes.value;
    merge_input_properties(input, {shown_marks});
  });

  function update_plot() {
    const {show, shown_marks, data, x_options, y_options} = input.value;
    // Update the plot; only recompute the figure if we show it.
    if (show) {
      plot_element = replace_element(plot_element, create_plot_element({shown_marks, data, x_options, y_options}));
    }
    d3.select(plot_details).style("display", show ? "initial" : "none");
  }

  input.addEventListener("input", function(){
    const {show, shown_marks} = input.value;
    
    // Update the dependent inputs; without triggering the event listeners so we don't loop.
    subtitle_checkbox.value = show ? [subtitle] : [];
    marks_checkboxes.value = shown_marks;

    update_plot();
  });

  input.addEventListener("plot-data", function(){
    update_plot();
  });

  return element;
};

function HorizontalStep(context, t) {
  this._context = context;
  this._t = t;
}

HorizontalStep.prototype = {
  areaStart: function() {
    this._line = 0;
  },
  areaEnd: function() {
    this._line = NaN;
  },
  lineStart: function() {
    this._x = this._y = NaN;
    this._point = 0;
  },
  lineEnd: function() {
    if (0 < this._t && this._t < 1 && this._point === 2) this._context.lineTo(this._x, this._y);
    if (this._line || (this._line !== 0 && this._point === 1)) this._context.closePath();
    if (this._line >= 0) this._t = 1 - this._t, this._line = 1 - this._line;
  },
  point: function(x, y) {
    x = +x, y = +y;
    switch (this._point) {
      case 0: this._point = 1; this._line ? this._context.lineTo(x, y) : this._context.moveTo(x, y); break;
      case 1: this._point = 2; // falls through
      default: {
        if (this._t <= 0) {
          this._context.lineTo(this._x, y);
          this._context.lineTo(x, y);
        } else {
          var x1 = this._x * (1 - this._t) + x * this._t;
          this._context.lineTo(x1, this._y);
          this._context.moveTo(x1, y);
        }
        break;
      }
    }
    this._x = x, this._y = y;
  }
};

export function horizontal_step(context) {
  return new HorizontalStep(context, 0.5);
}

export function horizontal_step_before(context) {
  return new HorizontalStep(context, 0);
}

export function horizontal_step_after(context) {
  return new HorizontalStep(context, 1);
}
