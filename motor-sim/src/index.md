---
toc: false
---

```js
import {md, note, link} from "./components/utils.js"
```


<div class="hero">
  <h1>3 Phase Motor</h1>
</div>



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
* [ ] hall sensor toggle angle (the hall sensor senses positive magnetic field, so it
toggles at almost 90 degrees difference from the rotor angle; the field is null at 90 degrees).
* [ ] rotor mass (so we can measure axial deflection when motor is )
* [ ] rotor axial restoration force constant



Measured/datasheet characteristics:
* Rotor moment of inertia for 10g shell of 5mm radius: 0.00000025 Kg*m^2.
* Forward diode voltage Vds = 0.72V (up to 1V); body-diode can withstand 4A continuous current.
* Continuous drain current 8.5A (at high ambient temperature 70C).
* Drain source ON resistance 18mΩ.
* Phase winding resistance 1.3Ω (2.6Ω across 2 phases).
* Shunt resistance 10mΩ.
* Driver turn on propagation delay ~300ns.
* Driver turn off propagation delay ~100ns.
* Driver automatic deadtime ~200ns.
* Phase inductance 0.1mH (0.2mH across 2 phases).

References:
* https://www.controleng.com/articles/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor/
* https://www.youtube.com/watch?v=EHYEQM1sA3o&list=PLaBr_WzeIAixidGwqfcrQlwKZX4RZ2E7D&pp=iAQB




<style>

.hero {
  display: flex;
  flex-direction: column;
  align-items: center;
  font-family: var(--sans-serif);
  margin: 4rem 0 8rem;
  text-wrap: balance;
  text-align: center;
}

.hero h1 {
  margin: 1rem 0;
  padding: 1rem 0;
  max-width: none;
  font-size: 14vw;
  font-weight: 900;
  line-height: 1;
  background: linear-gradient(30deg, var(--theme-foreground-focus), currentColor);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero h2 {
  margin: 0;
  max-width: 34em;
  font-size: 20px;
  font-style: initial;
  font-weight: 500;
  line-height: 1.5;
  color: var(--theme-foreground-muted);
}

@media (min-width: 640px) {
  .hero h1 {
    font-size: 90px;
  }
}

</style>
