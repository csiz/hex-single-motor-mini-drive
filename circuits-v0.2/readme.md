Hex Mini Drive Circuit
======================

Version v0.2

TODO for next circuit
---------------------

[ ] Maybe switch to a oscillation damping gate resistor of 3ohm. The AO4266E switching parameter uses 
Rgen = 3ohm, I think this is their recommended gate resistance.

[ ] Aaaargh, BOOT0 pin needs a pulldown resistor on the STM32G4!
[ ] VREF+ should be tied to VDDA...


[v] Think about connectors better. We should either drop or double down on USB-C. It would be very user friendly and the power delivery 
allows us to pick the 9-24V we need, but might be cumbersombe to control 24 devices. The SPI bus with slim connectors let's us keep it 
tight and in-house. Would we want a cheap single-sided board with USB-C and also a compact double sided assembly without USB at all.
... Let's go for double sided compact without USB, with the better stm32g4 with 5 opamps (so we can also ditch the opamp).
[v] USB-C footprint was correct with the cutouts. What we have now is incorrect!

[v] Need master circuit.
[v] Need flexi rotation I2C sensors.
[v] Via diameter was too small, should be at least 0.2/0.45


Need to implement in the new circuit
------------------------------------

Features of current version:
[v] Measure power line voltage.
[x] Re-pick voltage boost capacitor and resistor pair (using P FETS).
[v] Re-pick gate oscillation damping resistor for the inputs to the power MOSFET.
[v] Crystal oscilator and pick matching caps.
[v] Power regulator.
[v] USB-C connector maybe?
[v] Debugging connection with the trace line.
[v] 4 half bridge outputs
[v] 4 voltage inputs for the 4 outputs
[v] 1 voltage input for VCC
[v] 2 (preferably 4) current inputs from the 4 outputs; I chose 4.
[x] maybe 1 current input for total current. It would leave us with a floating ground which is iffy.
[v] SPI connection to main board. Not ~I2C~, it's too slow and consumes too much current. SPI will need
a chip select pin, but allows for parallel devices. SPI has hardware CRC calculation, seems useful.
[v] Spare I2C connection for the magnetic sensors.
[v] Status LED.

Lessons from previous version:
[x] Measure gate turn on characteristics of existing circuit...
[x] Maybe switch to a oscillation damping gate resistor of 3ohm. The AO4266E switching parameter uses 
Rgen = 3ohm, I think this is their recommended gate resistance.
[x] Definitely need to think about daisy chaining these, maybe use a signal buffer/transciever with an enable pin,
and/or mosfet switch for the chip power on. I2C to master board is probably the best and easiest way.
[v] Oopsies I flipped the sign of the V phase current measurement lines.
[x] The boost gate should maybe be connected to the driver side of the shunt resistor. At the moment we're measuring current used to charge the high gate.
[x] There's a summing option for the current measurments. Try outputing U, V, U+V, U+V+W using the 4th spare output.
[v] ! Think carefully about the shunt resistor placement, there's big interference going on if we measure during high side. Timing it around ground as
common mode voltage seems to fix it. Is that the only way to measure all current going through the motor coils?
[v] ! Measure voltage of the motor coils!
[x] There's a conflict between i2c and timer 3 channel 2. Choose a different GPIO for the red LED.
	I2C1 and TIM3_CH2 remapped
	Description
	When the following conditions are met:
	• I2C1 and TIM3 are clocked.
	• I/O port pin PB5 is configured as an alternate function output
	there is a conflict between the TIM3_CH2 signal and the I2C1 SMBA signal (even if SMBA is not used).
	In these cases the I/O port pin PB5 is set to 1 by default if the I/O alternate function output is selected and I2C1 is
	clocked. TIM3_CH2 cannot be used in output mode.
	Workaround
	To avoid this conflict, TIM3_CH2 can only be used in input mode.

[x] Double sided board. Not just yet.
[x] Drop voltage regulator and rely on master board for 3.3V; filter it though. We just needed to look for better 
components, there are cheap and small options that work well for us.
[x] Look for thinner IDC connector, and definitely SMD version for easier routing, use of space behind connector.
[v] Add voltage probes for the phase connectors so we can measure voltage both during driving and open connections.
We need to measure driving voltage to make sure our MOSFETS are turned on and giving us the voltage we command!
[v] Use Kelvin sense resistors (or pad hack) for the current measurement to avoid the unknown solder resistance.
https://www.analog.com/media/en/analog-dialogue/volume-46/number-2/articles/optimize-high-current-sensing-accuracy.pdf
[v] Don't drop the hall sensors, yet!
[v] The big problem with our measurements occurs when we switch from continuous 0 output from the low mosfet to mixed
output with the high mosfet. Any amount of mixed output seems to add an offset to our current measurement. Might be
worth putting the current sense only on the low mosfet branch.
[x] Choose a different mosfet driver to lower the minimum voltage under 5V. The undervoltage protection must be there
for a reason. Anyway, we chose 6V and up.
[v] Make it work with steppers... Neat!
