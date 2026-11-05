# Steady-State Open-Loop High-Frequency Micro-Oscillation Model

## Problem Setup & Physical Constraints
A 2-phase (x, y), single pole-pair, non-salient permanent magnet synchronous motor is driven open-loop by high-frequency balanced sinusoidal voltages (${tex`V_{\text{mag}}, \omega_p`}) under constant PWM/voltage settings. The rotor is unconstrained mechanically (zero friction, zero external load) but possesses a known inertial mass (J) and stable torque/back-EMF constant (${tex`K_t`}). 

When the probing angular frequency (${tex`\omega_p`}) is higher than the motor's mechanical break frequency, the rotor fails to entrain (synchronise) with the traveling wave. Instead, it enters a dynamic equilibrium, settling at a stable mean bias position (${tex`\theta_0`}) while executing low-amplitude, high-frequency mechanical micro-oscillations driven by the alternating quadrature current. 

The control loop samples the stator currents perfectly at the average current point every cycle with an arbitrary discrete PWM/sampling period (${tex`\Delta t`}). The discrete current updates are captured as ${tex`\Delta I_{x,t} = I_{x,t1} - I_{x,t}`} and ${tex`\Delta I_{y,t} = I_{y,t1} - I_{y,t}`}.

---

## Discrete State-Space Volt-Ampere Balance Equations

### 1. X-Axis Grouped Equation
```tex
\left(\begin{aligned} &\left[ V_{\text{mag}} + \sin(\theta_0) \cdot I_{\text{react}} \cdot R - I_{\text{avg}} \cdot R \cdot \frac{R}{\sqrt{R^2 + (\omega_p L)^2}} \right] \cos(\omega_p t) \\ + &\left[ \sin(\theta_0) \cdot I_{\text{react}} \cdot (\omega_p L) - I_{\text{avg}} \cdot R \cdot \frac{\omega_p L}{\sqrt{R^2 + (\omega_p L)^2}} \right] \sin(\omega_p t) \\ - &V_{\text{mag}} \left(\frac{\Delta I_{x,t}}{\Delta I_{\text{avg}}}\right) = 0 \end{aligned} \right)
```

### 2. Y-Axis Grouped Equation
```tex
\left(\begin{aligned} &\left[ \cos(\theta_0) \cdot I_{\text{react}} \cdot R + I_{\text{avg}} \cdot R \cdot \frac{\omega_p L}{\sqrt{R^2 + (\omega_p L)^2}} \right] \cos(\omega_p t) \\ + &\left[ V_{\text{mag}} + \cos(\theta_0) \cdot I_{\text{react}} \cdot (\omega_p L) - I_{\text{avg}} \cdot R \cdot \frac{R}{\sqrt{R^2 + (\omega_p L)^2}} \right] \sin(\omega_p t) \\ - &V_{\text{mag}} \left(\frac{\Delta I_{y,t}}{\Delta I_{\text{avg}}}\right) = 0 \end{aligned} \right)
```

---

## Unified Parameter and Scaler Components

### The Mechanical Reaction Current Scale (${tex`I_{\text{react}}`})
```tex
\left(I_{\text{react}} = I_{\text{avg}} \cdot \left[ \frac{K_t^2 \Delta t}{J \sqrt{R^2 + (\omega_p L)^2}} \right] \cdot \left[ \frac{1}{\omega_p \Delta t} \right] \right)
```

### The Average Electrical Current Scale (${tex`I_{\text{avg}}`})
```tex
\left(I_{\text{avg}} = \frac{V_{\text{mag}}}{\sqrt{R^2 + (\omega_p L)^2}} \right)
```

### The Average Discrete Current Step Scale (${tex`\Delta I_{\text{avg}}`})
```tex
\Delta I_{\text{avg}} = \frac{V_{\text{mag}} \Delta t}{L}
```

---

## Parameter Identification & The "Resistance Spike" Mechanism
When parameter identification loops process these signals under the assumption of a rigidly locked rotor (${tex`\omega = 0, J \to \infty`}), the mechanical back-EMF contributions from the micro-vibrations do not drop to zero. Instead, they project directly onto the orthogonal ${tex`\cos(\omega_p t)`} and ${tex`\sin(\omega_p t)`} base vectors.

* **In-Phase Distortions:** The terms containing ${tex`I_{\text{react}} \cdot R`} map perfectly in phase with the expected voltage drops caused by stator winding resistance.
* **Quadrature Distortions:** The terms containing ${tex`I_{\text{react}} \cdot (\omega_p L)`} shift by 90 electrical degrees, mapping directly over the expected inductive voltage drops (${tex`L \frac{dI}{dt}`}).

Because ${tex`I_{\text{react}}`} scales inversely with the probing frequency squared (${tex`\sim 1/\omega_p^2`}), lower frequencies pass significant mechanical energy back into the electrical system via back-EMF. Parameter estimators interpret this incoming mechanical energy as excess real power dissipation, resulting in a pronounced, virtual **resistance spike** that rapidly attenuates to zero as ${tex`\omega_p`} scales past the mechanical tracking threshold of the rotor's inertia.


# The Open-Loop Parameter Separation Trick

By treating a vibrating, unconstrained rotor as a locked system during low-frequency open-loop probing, an estimator calculates four corrupted parameters: ${tex`\hat{R}_x`}, ${tex`\hat{R}_y`}, ${tex`\hat{L}_x`}, and ${tex`\hat{L}_y`}. 

Because the mechanical back-EMF distorts these values through a single spatial vector (${tex`\theta_0`}), this unique asymmetrical footprint contains enough information to simultaneously decode the true electrical parameters (${tex`R_{\text{true}}, L_{\text{true}}`}), the static rotor position (${tex`\theta_0`}), and the electromechanical torque capability (${tex`K_t^2/J`}) without an encoder or a datasheet.

---

## 1. The Corrupted Measurement Framework

Your independent dual-axis parameter estimator calculates four values that blend true copper attributes with the dimensionless mechanical reaction scaling factor ${tex`V_{\text{m}} = I_{\text{react}}/I_{\text{avg}}`}:

```tex
\hat{R}_x = R_{\text{true}} + \sin(\theta_0) \cdot V_{\text{m}} \cdot (\omega_p L_{\text{true}})
```

```tex
\hat{R}_y = R_{\text{true}} + \cos(\theta_0) \cdot V_{\text{m}} \cdot R_{\text{true}}
```

```tex
\hat{L}_x = L_{\text{true}} + \sin(\theta_0) \cdot V_{\text{m}} \cdot \left(\frac{R_{\text{true}}}{\omega_p}\right)
```

```tex
\hat{L}_y = L_{\text{true}} + \cos(\theta_0) \cdot V_{\text{m}} \cdot L_{\text{true}}
```

---

## 2. Step-by-Step Extraction Sequence

### Step A: Isolate the True Electrical Constraint
Multiplying the inductance values by the known probing frequency ${tex`\omega_p`} shifts them to the impedance domain (ohms). Because of the cross-coupling layout of the back-EMF, subtracting the cross-axis parameter pairs completely dissolves the mechanical distortion vector:

```tex
R_{\text{true}} - \omega_p L_{\text{true}} = \hat{R}_x - \omega_p \hat{L}_y
```

This provides a direct, uncorrupted linear tracking relationship between your true copper winding resistance and inductance.

### Step B: Deduce the Unknown Rotor Angle (${tex`\theta_0`})
By separating the parameter differences between the two phases, the unknown mechanical reaction amplitude ${tex`V_{\text{m}}`} cancels out entirely. Evaluating the ratio of these spatial cross-axis differences isolates the static rotor alignment angle:

```tex
	\theta_0 = \arctan \left[ \left( \frac{\hat{R}_x - \hat{R}_y}{\omega_p \hat{L}_x - \omega_p \hat{L}_y} \right) \cdot \left( \frac{\hat{R}_y}{\hat{R}_x} \right) \right]
```

### Step C: Resolve True ${tex`R`} and ${tex`L`}
With the spatial angle ${tex`\theta_0`} computed, the parameter system collapses back into a basic set of linear equations. Substituting ${tex`\theta_0`} reveals the uncorrupted winding resistance:

```tex
R_{\text{true}} = \frac{\hat{R}_y \cdot \sin(\theta_0) - \hat{R}_x \cdot \cos(\theta_0) \cdot \left(\frac{R_{\text{true}}}{\omega_p L_{\text{true}}}\right)}{\sin(\theta_0) - \cos(\theta_0)}
```

Once ${tex`R_{\text{true}}`} is resolved, the true winding inductance is calculated instantly from the constraint established in Step A:

```tex
L_{\text{true}} = \frac{R_{\text{true}} - (\hat{R}_x - \omega_p \hat{L}_y)}{\omega_p}
```

---

## 3. Deducing the Electromechanical Group (${tex`K_t^2/J`})

Now that the true uncorrupted electrical parameters are isolated, you can calculate the raw mechanical reaction scale ${tex`V_{\text{m}}`} directly from your original data stream:

```tex
V_{\text{m}} = \frac{\hat{R}_y - R_{\text{true}}}{\cos(\theta_0) \cdot R_{\text{true}}}
```

Expanding ${tex`V_{\text{m}}`} back into its continuous structural components extracts the combined electromechanical tracking constant of your motor's torque-to-inertia ratio:

```tex
\frac{K_t^2}{J} = \omega_p \cdot \sqrt{R_{\text{true}}^2 + (\omega_p L_{\text{true}})^2} \cdot V_{\text{m}}
```
