import * as Inputs from "@observablehq/inputs";
import {Generators} from "observablehq:stdlib";

import {html} from "htl";

import _ from "lodash";

import {set_stored, get_stored_or_default} from "./local_storage.js";

export function enabled_checkbox(data, {...options}){
  return Inputs.checkbox(data, {value: data, ...options});
}

export async function * any_checked_input(checkbox_input){
  for await (const checked of Generators.input(checkbox_input)) {
    yield checked.length > 0;
  }
}

export function set_input_value(input, value) {
  input.value = value;
  input.dispatchEvent(new Event("input", {bubbles: true}));
}

export function merge_input_value(input, value) {
  input.value = {...input.value, ...value};
  input.dispatchEvent(new Event("input", {bubbles: true}));
}


export function autosave_inputs(inputs, delay_millis = 100){
  Object.entries(inputs).forEach(([key, input]) => {
    input.addEventListener("input", _.debounce(() => {
      set_stored(key, input.value);
    }, delay_millis));
    set_input_value(input, get_stored_or_default(key, input.value));
  });
}
