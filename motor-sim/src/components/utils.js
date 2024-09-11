import {html} from "npm:htl";
import {require} from "d3-require";
import {md as md_loader} from "npm:@observablehq/stdlib/src/md.js";


export const md = await md_loader(require);

export function note (text) {
	return html`<div class="tooltip">*<span class="tooltiptext">${text}</span></div>`;
}

export function link (url) {
	return html`<a href="${url}" target="_blank">${url}</a>`;
}

