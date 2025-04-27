import * as Plot from "@observablehq/plot";
import * as Inputs from "@observablehq/inputs";
import {Generators} from "observablehq:stdlib";

import {html} from "htl";

import {set_stored, get_stored_or_default} from "./local_storage.js";

export function enabled_checkbox(data, {...options}){
  return Inputs.checkbox(data, {value: data, ...options});
}

export function autosave_inputs(inputs){
  Object.entries(inputs).forEach(([key, input]) => {
    input.addEventListener("input", () => {
      set_stored(key, input.value);
    });
    input.value = get_stored_or_default(key, input.value);
  });
}

export async function * any_checked_input(checkbox_input){
  for await (const checked of Generators.input(checkbox_input)) {
    yield checked.length > 0;
  }
}