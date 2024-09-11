---
title: Understanding 3-phase motor control
theme: dashboard
---

```js
import {md, note, link} from "./components/utils.js"
```

```js

const phi = d3.ticks(0, 2 * Math.PI, 1000);
const phases = phi.map(t => {
  const u = Math.sin(t);
  const v = Math.sin(t + 2 * Math.PI / 3);
  const w = Math.sin(t + 4 * Math.PI / 3);

  const max = Math.max(u, v, w);
  const min = Math.min(u, v, w);
  const max_adj = 1 - max;
  const min_adj = -1 - min;
  const adj = max_adj/2 + min_adj/2;

  const adj_u = u + adj;
  const adj_v = v + adj;
  const adj_w = w + adj;
  return {u, v, w, adj, max, min, max_adj, min_adj, adj_u, adj_v, adj_w};
});


display(Plot.plot({marks: [
  Plot.lineY(phases, {x: phi, y: "u", stroke: 'red', label: 'U'}),
  Plot.lineY(phases, {x: phi, y: "v", stroke: 'green', label: 'V'}),
  Plot.lineY(phases, {x: phi, y: "w", stroke: 'blue', label: 'W'}),
  Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
]}))

display(Plot.plot({marks: [
  Plot.lineY(phases, {x: phi, y: "adj", stroke: 'black', label: 'Adjustment'}),
  Plot.lineY(phases, {x: phi, y: "max_adj", stroke: 'black', label: 'Max', strokeDasharray: '2'}),
  Plot.lineY(phases, {x: phi, y: "min_adj", stroke: 'black', label: 'Min', strokeDasharray: '2'}),
  Plot.lineY(phases, {x: phi, y: "adj_u", stroke: 'red', label: 'U'}),
  Plot.lineY(phases, {x: phi, y: "adj_v", stroke: 'green', label: 'V'}),
  Plot.lineY(phases, {x: phi, y: "adj_w", stroke: 'blue', label: 'W'}),
  Plot.gridX({interval: Math.PI / 3, stroke: 'black', strokeWidth : 2}),
]}))

```


Motor model parameters
----------------------

* [ ] rotor inertial mass
* [ ] phase inductance L
* [ ] phase resistance R
* [ ] motor Kv constant
* [ ] hysterisis angle alpha ${note(html`
	Apparently the phase delay between the motor flux and stator flux is constant and
	depends on the hysterisis loop characteristic of the rotor construction.
	${link("https://www.ijerd.com/paper/vol12-issue5/Version-1/K1257683.pdf")}
`)}
* [ ] hysterisis friction torque
	the hysterisis torque is also constant
* [ ] static friction torque (fixed torque at standstill)
* [ ] dynamic friction torque (fixed torque when rotating)
* [ ] resistance friction torque (this one is proportional to rotation speed)
* [ ] battery internal resistance
* [ ] mosfet resistance
* [ ] mosfet reverse diode voltage drop
* [ ] hall sensor toggle flux
* [ ] rotor mass (so we can measure axial deflection when motor is )
* [ ] rotor axial restoration force constant



Measured/datasheet characteristics:
* Forward diode voltage Vds = 0.72V (up to 1V); body-diode can withstand 4A continuous current.
* Continuous drain current 8.5A (at high ambient temperature 70C).
* Drain source ON resistance 18mΩ.
* Phase winding resistance 1.3Ω (2.6Ω across 2 phases).
* Shunt resistance 10mΩ.
* Driver turn on propagation delay ~300ns.
* Driver turn off propagation delay ~100ns.
* Driver automatic deadtime ~200ns.

References:
* https://www.controleng.com/articles/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor/
* https://www.youtube.com/watch?v=EHYEQM1sA3o&list=PLaBr_WzeIAixidGwqfcrQlwKZX4RZ2E7D&pp=iAQB



<style>
.tooltip {
  position: relative;
  display: inline-block;
  border-bottom: 1px dotted black;
}

.tooltip .tooltiptext {
  visibility: hidden;
  width: 720px;
  background-color: black;
  color: #fff;
  text-align: center;
  border-radius: 6px;
  padding: 5px 0;
  
  /* Position the tooltip */
  position: absolute;
  z-index: 1;
  top: 100%;
  left: 50%;
  margin-left: -240px;
}

.tooltip:hover .tooltiptext {
  visibility: visible;
}
</style>