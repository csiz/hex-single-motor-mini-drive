import _ from "lodash";
import {html} from "htl";
import * as Inputs from "@observablehq/inputs";
import * as d3 from "d3";

import {merge_input_value} from "./input_utils.js";


export function plot_lines({
  data = [],
  x_domain = undefined,
  y_domain = undefined,

  width = 928, height = 600, 
  marginTop = 20, marginRight = 10, marginBottom = 30, marginLeft = 50,

  x_format = ((x) => x.toFixed(3).padStart(9)),
  y_format = ((y) => y.toFixed(3).padStart(9)),
  x_label, y_label,
  x, 
  subtitle, description,
  channels,
  curve = d3.curveLinear,
  default_shown_marks = undefined,
  default_show = true,
  include_crosshair = true,
}){
  const dimensions = {width, height, marginTop, marginRight, marginBottom, marginLeft};

  const initial_value = {
    show: default_show,
    shown_marks: default_shown_marks ?? channels.map(({label}) => label),
  };
  
  const description_element = html`<p>${description}</p>`;

  // First, make the title into a checkbox to toggle the plot on and off.
  const subtitle_checkbox = Inputs.checkbox([subtitle], {
    value: initial_value.show ? [subtitle] : [],
    format: (subtitle) => html`<h4 style="min-width: 20em; font-size: 1.5em; font-weight: normal;">${subtitle}</h4>`,
  });
  
  // Then, make the marks into checkboxes to toggle them on and off.
  const marks_checkboxes = Inputs.checkbox(
    channels.map(({label}) => label), 
    {
      value: initial_value.shown_marks,
      label: "Display:",
      format: (label, i) => html`<span style="border-bottom: solid 3px ${channels[i].color}; margin-bottom: -3px;">${label}</span>`,
    },
  );

    // Create the SVG container.
  const svg = d3.create("svg")
    .attr("width", width)
    .attr("height", height)
    .attr("viewBox", [0, 0, width, height])
    .attr("style", `max-width: 100%; height: auto; overflow: visible;`);

  // Add the horizontal axis.
  const horizontal_axis = svg.append("g")
    .attr("transform", `translate(0,${height - marginBottom})`)
    .call(g => g.append("text")
      .attr("x", width - marginRight)
      .attr("y", marginBottom)
      .attr("fill", "currentColor")
      .attr("text-anchor", "end")
      .text(`→ ${x_label}`));

  // Add the vertical axis.
  const vertical_axis = svg.append("g")
    .attr("transform", `translate(${marginLeft},0)`)
    .call(g => g.append("text")
      .attr("x", -marginLeft)
      .attr("y", 10)
      .attr("fill", "currentColor")
      .attr("text-anchor", "start")
      .text(`↑ ${y_label}`));



  const plot_root = svg.append("g")
    .attr("clip-path", `inset(${marginTop - 2}px ${marginRight - 2}px ${marginBottom - 2}px ${marginLeft - 2}px) view-box`)
    .attr("fill", "none")
    .attr("stroke-width", 1.5)
    .attr("stroke-linejoin", "round")
    .attr("stroke-linecap", "round");


  // Create the positional scales.
  const x_scale = d3.scaleLinear()
    .range([marginLeft, width - marginRight]);

  const y_scale = d3.scaleLinear()
    .range([height - marginBottom, marginTop]);

    
  const plot_contents = html`<div>${description_element}${marks_checkboxes}${svg.node()}</div>`;


  // Build the final figure.
  
  const result = html`<figure>${subtitle_checkbox}${plot_contents}</figure>`;


  result.value = initial_value;

  result.plot_data = {data, x_domain, y_domain};

  result.shown_data = undefined;

  result.update = (draw_data) => {
    if (!result.value.show) return;

    result.plot_data = {...result.plot_data, ...draw_data};

    // Get latest values.
    const {data, x_domain, y_domain} = result.plot_data;


    if (!data || data.length === 0) {
      plot_root.selectAll("g").remove();
      return;
    }

    const shown_channels = channels.filter(({label}) => result.value.shown_marks.includes(label));
    
    const x_values = data.map((d, i, data) => pick_value(x, d, i, data));

    // Update the scales with potentionally new domains.
    if (x_domain) {
      x_scale.domain(x_domain);
    } else {
      x_scale.domain(d3.extent(x_values)).nice();
    }

    if (y_domain) {
      y_scale.domain(y_domain);
    } else {
      const min_y_value = d3.min(shown_channels, ({y}) => d3.min(data, (d, i, data) => pick_value(y, d, i, data)));
      const max_y_value = d3.max(shown_channels, ({y}) => d3.max(data, (d, i, data) => pick_value(y, d, i, data)));
      y_scale.domain([min_y_value, max_y_value]).nice();
    }


    horizontal_axis.call(d3.axisBottom(x_scale).ticks(width / 80).tickSizeOuter(0));
    vertical_axis.call(d3.axisLeft(y_scale).ticks(height / 40).tickSizeOuter(0)).call(g => g.select(".domain").remove());

    // Add the common x selector and data to each channel; so we can use a generalized function to draw them.
    result.shown_data = shown_channels.map(channel => ({...channel, x, ...result.plot_data, x_scale, y_scale, curve}));

    // Redraw channels.
    plot_root.selectAll("g")
      .data(result.shown_data)
      .join("g").each(function(channel_data){
        // Clear previous glyphs.
        d3.select(this).selectChildren().remove();

        // Draw the lines.
        draw_line.call(this, channel_data);

        channel_data.draw_extra?.call(this, channel_data);
      });
  };

  
  result.update(result.plot_data);
  
  
  subtitle_checkbox.addEventListener("input", (event) => {
    const show = subtitle_checkbox.value.length > 0;
    merge_input_value(result, {show});
    event.stopPropagation();
  });
  
  marks_checkboxes.addEventListener("input", (event) => {
    const shown_marks = marks_checkboxes.value;
    merge_input_value(result, {shown_marks});
    event.stopPropagation();
  });
  
  result.addEventListener("input", function(){
    const {show, shown_marks} = result.value;
    
    // Update the dependent inputs; without triggering the event listeners so we don't loop.
    subtitle_checkbox.value = show ? [subtitle] : [];
    marks_checkboxes.value = shown_marks;
    
    d3.select(plot_contents).style("display", show ? "initial" : "none");
    
    result.update(result.plot_data);
  });

  // Add the crosshair if requested.
  if (include_crosshair) {
    add_crosshair(result, {svg, plot_root, x_format, y_format, x_label, y_label, ...dimensions});
  }

  return result;
}

export function add_crosshair(plot, {svg, plot_root, x_format, y_format, x_label, y_label, width, height, marginLeft, marginRight, marginTop, marginBottom}){
  const font_size = 14;

  // Add an invisible layer for the interactive tip.
  const tip = svg.insert("g", ":first-child")
    .attr("display", "none");

  const circle = tip.append("circle")
    .attr("fill", "none")
    .attr("stroke-width", 2)
    .attr("r", 3);

    
  const crosshair_x = tip.append("line")
    .attr("stroke-width", 2)
    .attr("stroke-opacity", 0.2)
    .attr("y1", 0)
    .attr("y2", height);
    
  const crosshair_y = tip.append("line")
    .attr("stroke-width", 2)
    .attr("stroke-opacity", 0.2)
    .attr("x1", marginLeft)
    .attr("x2", width);

  const mini_size = 10;
    
  const hover_x = tip.append("text")
    .attr("style", `font: ${mini_size}px sans-serif; font-family: monospace; stroke: #FFFFFF; stroke-width: 5px; paint-order: stroke;`)
    .attr("text-anchor", "middle")
    .attr("vertical-align", "bottom")
    .attr("y", height - marginBottom + mini_size + 5);

  const hover_y = tip.append("text")
    .attr("style", `font: ${mini_size}px sans-serif; font-family: monospace; stroke: #FFFFFF; stroke-width: 5px; paint-order: stroke;`)
    .attr("text-anchor", "end")
    .attr("dominant-baseline", "middle")
    .attr("x", marginLeft - 5);


  const hover_text = tip.append("text")
    .attr("style", `font: ${font_size}px sans-serif; font-family: monospace; stroke: #FFFFFF; stroke-width: 5px; paint-order: stroke;`)
    .attr("text-anchor", "end")
    .attr("x", width - marginRight)
    .attr("y", marginTop - 5);

  
  // When the pointer moves, find the closest point, update the interactive tip, and highlight
  // the corresponding line.
  svg.on("pointermove", (event) => {
    const [xm, ym] = d3.pointer(event);
    if (!plot.shown_data || plot.shown_data.length == 0) return;

    const [x_i, y_i] = min_index_2d(plot.shown_data.map(({x, y, x_scale, y_scale, data}) => {
      return data.map((d, i) => {
        const coord_x = x_scale(pick_value(x, d, i, data));
        const coord_y = y_scale(pick_value(y, d, i, data));
        return Math.hypot(coord_x - xm, coord_y - ym);
      });
    }));

    const {x, y, color, label, x_scale, y_scale, data} = plot.shown_data[y_i];
    
    // Mute all other lines and raise the selected one.
    plot_root.selectAll("g").attr("opacity", (channel) => channel.label === label ? null : 0.3);
      

    const value_x = pick_value(x, data[x_i], x_i, data);
    const value_y = pick_value(y, data[x_i], x_i, data);

    const formatted_x = x_format(value_x);
    const formatted_y = y_format(value_y);
    
    const coord_x = x_scale(value_x);
    const coord_y = y_scale(value_y);
    
    const text = `${x_label}: ${formatted_x} | ${y_label}: ${formatted_y} | ${label.padStart(20)}`;

    tip.attr("display", null);
    
    circle
      .attr("cx", coord_x)
      .attr("cy", coord_y)
      .attr("stroke", color);

    hover_y
      .text(formatted_y)
      .attr("y", coord_y)
      .attr("fill", color);

    hover_x
      .text(formatted_x)
      .attr("x", coord_x)
      .attr("fill", color);

    crosshair_x
      .attr("stroke", color)
      .attr("x1", coord_x)
      .attr("x2", coord_x);
      
    crosshair_y
      .attr("stroke", color)
      .attr("y1", coord_y)
      .attr("y2", coord_y);

    hover_text
      .text(text)
      .attr("fill", color);

  });

  svg.on("pointerleave", () => {
    plot_root.selectAll("g").attr("opacity", null);
    tip.attr("display", "none");
  });
}


export function draw_line({data, x, y, x_scale, y_scale, curve, color}) {
  const make_line = d3.line()
    .x((d, i, data) => x_scale(pick_value(x, d, i, data)))
    .y((d, i, data) => y_scale(pick_value(y, d, i, data)))
    .curve(curve)
    .defined((d, i, data) => d != null && not_nan_or_null(pick_value(y, d, i, data)));

  return d3.select(this).append("path")
    .style("stroke", color)
    .attr("d", make_line(data));
}

export function draw_area({data, x, y0, y1, x_scale, y_scale, curve, color}) {
  const make_area = d3.area()
    .x((d, i, data) => x_scale(pick_value(x, d, i, data)))
    .y0((d, i, data) => y_scale(pick_value(y0, d, i, data)))
    .y1((d, i, data) => y_scale(pick_value(y1, d, i, data)))
    .curve(curve)
    .defined((d, i, data) => d != null && not_nan_or_null(pick_value(y0, d, i, data)) && not_nan_or_null(pick_value(y1, d, i, data)));

  return d3.select(this).append("path")
    .style("fill", color)
    .attr("d", make_area(data));
}

export function setup_faint_area({y0, y1}){
  return function(draw_data){
    draw_area.call(this, {...draw_data, y0, y1}).style("fill-opacity", 0.2);
  };
}

export function draw_v_line({x_value, x_scale, y_scale, color}){
  return d3.select(this).append("line")
    .attr("stroke-width", 3)
    .attr("stroke", color)
    .attr("y1", y_scale.range()[0])
    .attr("x1", x_scale(x_value))
    .attr("y2", y_scale.range()[1])
    .attr("x2", x_scale(x_value));
}

export function setup_v_line({x_value, dasharray = undefined}){
  return function(draw_data){
    draw_v_line.call(this, {
      ...draw_data, 
      x_value: x_value(draw_data),
    }).attr("stroke-dasharray", dasharray ?? null);
  };
}


// Custom d3.curve to plot unconnected horizontal steps for each data point.
class HorizontalStep {
  constructor(context, t) {
    this._context = context;
    this._t = t;
  }

  areaStart() {
    this._line = 0;
  }

  areaEnd() {
    this._line = NaN;
  }

  lineStart() {
    this._x = this._y = NaN;
    this._point = 0;
  }

  lineEnd() {
    if (0 < this._t && this._t < 1 && this._point === 2) this._context.lineTo(this._x, this._y);
    if (this._line || (this._line !== 0 && this._point === 1)) this._context.closePath();
    if (this._line >= 0) this._t = 1 - this._t, this._line = 1 - this._line;
  }

  point(x, y) {
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
}

export function horizontal_step(context) {
  return new HorizontalStep(context, 0.5);
}

export function horizontal_step_before(context) {
  return new HorizontalStep(context, 0);
}

export function horizontal_step_after(context) {
  return new HorizontalStep(context, 1);
}

// Utility functions
// -----------------

// Get the 2d coordiates of the minimum value in a 2d array.
function min_index_2d(array2d) {
  const min_indexes = array2d.map((array) => d3.minIndex(array));
  const y_index = d3.minIndex(min_indexes, (min_i, i) => array2d[i][min_i]);
  const x_index = min_indexes[y_index];
  return [x_index, y_index];
}

// Pick a value by key, function or constant. The function is called with the usual mapping arguments.
function pick_value(value, d, i, data) {
  if (_.isFunction(value)) {
    return value(d, i, data);
  } else if (_.isString(value)) {
    return d[value];
  } else {
    return value;
  }
}

// Check if a value is neither NaN nor null.
function not_nan_or_null(d) {
  return d != null && !isNaN(d);
}
