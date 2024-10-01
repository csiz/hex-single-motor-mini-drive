---
title: 3 Phase Tricks
theme: dashboard
---

The voltages of the 3 phases are normally sinusoidal but the important metric is the
voltage difference between the phases. Notice that the minimum and maximum voltages
don't occur at the same time for all phases.

```js
display(Plot.plot({
  y: {domain: [-1.0, 1.0]},
  marks: [
    Plot.lineY(phases, {x: phi, y: "u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: phi, y: "v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: phi, y: "w", stroke: 'blue', label: 'W'}),
    Plot.lineY(phases, {x: phi, y: "max_adj", stroke: 'black', label: 'Max', strokeDasharray: '5 10', strokeWidth: 3}),
    Plot.lineY(phases, {x: phi, y: "min_adj", stroke: 'black', label: 'Min', strokeDasharray: '5 10', strokeWidth: 3}),
    Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
}))
```
```js
display(Plot.plot({
  y: {domain: [-2.0, 2.0]},
  marks: [
    Plot.lineY(phases, {x: phi, y: "uv", stroke: 'yellow', label: 'U-V'}),
    Plot.lineY(phases, {x: phi, y: "vw", stroke: 'cyan', label: 'V-W'}),
    Plot.lineY(phases, {x: phi, y: "wu", stroke: 'magenta', label: 'W-U'}),
    Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
}))
```


A convenient trick is to adjust the phases so that the minimum voltage is always zero
for some phase. The phase voltages below are equivalent to those above. Notice that
we use 86% of the voltage range after the adjustment. We can thus scale our voltages
by 1.16 in order to use the full range and get 15% more effective voltage (in the 
rotating phasor frame).

```js
display(Plot.plot({
  y: {domain: [0, 2.0]},
  marks: [
    Plot.lineY(phases, {x: phi, y: "adj", stroke: 'black', label: 'Adjustment'}),
    Plot.lineY(phases, {x: phi, y: "adj_u", stroke: 'red', label: 'U'}),
    Plot.lineY(phases, {x: phi, y: "adj_v", stroke: 'green', label: 'V'}),
    Plot.lineY(phases, {x: phi, y: "adj_w", stroke: 'blue', label: 'W'}),
    Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
    Plot.gridY({interval: 0.5, stroke: 'black', strokeWidth : 2}),
  ]
}))
```

<!-- Setup -->

```js
const phi = d3.ticks(0, 2 * Math.PI, 1000);
const phases = phi.map(t => {
  const u = Math.sin(t);
  const v = Math.sin(t + 2 * Math.PI / 3);
  const w = Math.sin(t + 4 * Math.PI / 3);

  const uv = u - v;
  const vw = v - w;
  const wu = w - u;

  const max = Math.max(u, v, w);
  const min = Math.min(u, v, w);
  const max_adj = max;
  const min_adj = min;
  const adj = - min_adj;

  const adj_u = u + adj;
  const adj_v = v + adj;
  const adj_w = w + adj;
  return {u, v, w, adj: -adj, max, min, max_adj, min_adj, adj_u, adj_v, adj_w, uv, vw, wu};
});
```
