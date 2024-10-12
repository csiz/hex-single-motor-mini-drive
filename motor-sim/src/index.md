---
toc: false
---

```js
import {note, link} from "./components/utils.js"
```

<div class="hero">

3 Phase Permanent Magnet Motor
=============================


Motor model parameters
----------------------

* [✓] rotor inertial mass
* [✓] phase inductance L
* [✓] phase resistance R
* [✓] motor Kv constant
* [ ] hysterisis modeling?
* [✓] static friction torque (fixed torque at standstill)
* [✓] dynamic friction torque (fixed torque when rotating)
* [✓] resistance friction torque (this one is proportional to rotation speed)
* [✓] battery internal resistance
* [✓] mosfet resistance
* [✓] mosfet reverse diode voltage drop
* [✓] hall sensor toggle angle (the hall sensor senses positive magnetic field, so it
toggles at almost 90 degrees difference from the rotor angle; the field is null at 90 degrees).
* [✓] rotor mass (so we can measure axial deflection when motor is )
* [✓] rotor axial restoration force constant
* [ ] cogging torque ripple



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
* Understanding the effect of PWM when controlling a brushless dc motor: https://www.controleng.com/articles/understanding-the-effect-of-pwm-when-controlling-a-brushless-dc-motor/
* Jantzen Lee - The Physics behind how motors work. Understanding motors (Episode 1): https://www.youtube.com/watch?v=EHYEQM1sA3o&list=PLaBr_WzeIAixidGwqfcrQlwKZX4RZ2E7D&pp=iAQB
* Texas Instruments - Field Oriented Control of Permanent Magnet Motors: https://www.youtube.com/watch?v=cdiZUszYLiA
* Clarke transformation: https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation
* Direct-quadrature-zero transformation: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation
* Coordinate Transform in Motor Control (shows the math workings to get the unitary Clarke transform): https://www.infineon.com/dgdl/Infineon-AN205345_Coordinate_Transform_in_Motor_Control-ApplicationNotes-v03_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0cff06bb58ed
* 3-Phase PM Synchronous Motor Torque Vector Control Using 56F805: https://www.nxp.com/docs/en/reference-manual/DRM018.pdf
* PMSM Permanent magnet synchronous motor with sinusoidal flux distribution: https://uk.mathworks.com/help/sps/ref/pmsm.html
* Electromotive forceL https://en.wikipedia.org/wiki/Electromotive_force
* Counter-electromotive force: https://en.wikipedia.org/wiki/Counter-electromotive_force
* Magnetic flux https://en.wikipedia.org/wiki/Magnetic_flux
* Magnetic flux density (magnetic field): https://en.wikipedia.org/wiki/Magnetic_field
* Magnetic moment https://en.wikipedia.org/wiki/Magnetic_moment
* Faraday induction law: https://en.wikipedia.org/wiki/Faraday%27s_law_of_induction
* Solenoid: https://en.wikipedia.org/wiki/Solenoid


</div>