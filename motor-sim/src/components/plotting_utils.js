import _ from "lodash";
import {html} from "htl";
import {localStorage, get_stored_or_default} from "./local_storage.js";
import {Mutable} from "observablehq:stdlib";
import * as Plot from "@observablehq/plot";
import * as Inputs from "@observablehq/inputs";
import {enabled_checkbox, autosave_inputs, any_checked_input} from "./input_utils.js";


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
    data, 
    store_id,
    width, height,
    x_options, y_options,
    x_label, y_label,
    subtitle, description,
    mark_function = (selected_data, options) => Plot.line(selected_data, {...options}),
    x, 
    channels,
    grid_marks = [],
    other_marks = [],
    curve = undefined,
  } = options;

  let {selection} = options;

  selection = selection ?? get_stored_or_default(store_id, {
    show: true,
    shown_marks: [...channels.map(({y}) => y), "grid"],
  });

  let result = Mutable(create_element(selection));

  function update_selection(new_selection){
    selection = {...selection, ...new_selection};
    // Store the selection in local storage.
    localStorage.setItem(store_id, JSON.stringify(selection));
    // Update the plot.
    result.value = create_element(selection);
  }

  function create_element(selection){

    // First, make the title into a checkbox to toggle the plot on and off.
    const subtitle_checkbox = Inputs.checkbox([subtitle], {
      value: selection.show ? [subtitle] : [],
      format: (subtitle) => html`<h4 style="min-width: 20em; font-size: 1.5em; font-weight: normal;">${subtitle}</h4>`,
    });

    subtitle_checkbox.addEventListener("input", function(event){
      const show = subtitle_checkbox.value.length > 0;
      update_selection({show});
    });

    if (!selection.show) {
      return html`<div>${subtitle_checkbox}</div>`;
    }

    const description_element = html`<p>${description}</p>`;

    const checkbox_y_to_label = Object.fromEntries([...channels.map(({y, label}) => [y, label]), ["grid", "Grid"]]);
    const checkbox_y_to_color = Object.fromEntries([...channels.map(({y, color}) => [y, color]), ["grid", "grey"]]);

    // Then, make the marks into checkboxes to toggle them on and off.
    const marks_checkboxes = Inputs.checkbox(
      [...channels.map(({y}) => y), "grid"], 
      {
        value: selection.shown_marks,
        label: "Display:",
        format: (y) => html`<span style="border-bottom: solid 3px ${checkbox_y_to_color[y]}; margin-bottom: -3px;">${checkbox_y_to_label[y]}</span>`,
      },
    );

    marks_checkboxes.addEventListener("input", function(event){
      const shown_marks = marks_checkboxes.value;
      update_selection({shown_marks});
    });

    const selected_channels = channels.filter(({y}) => selection.shown_marks.includes(y));
    const selected_data = expand_z_channels({data, x_label, y_label, x, channels: selected_channels});

    const plot_figure = Plot.plot({
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
        ...other_marks.map(mark => _.isFunction(mark) ? mark(selected_data, {x: x_label, y: y_label, z: "z"}) : mark),
      ],
    });

    return html`<div>${subtitle_checkbox}${description_element}${marks_checkboxes}${plot_figure}</div>`;
  }

  return result;
}

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
