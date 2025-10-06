Need to implement in the new circuit
------------------------------------

[ ] Measure power line voltage.
[ ] Re-pick voltage boost capacitor and resistor pair.
[ ] Re-pick gate oscillation damping resistor for the inputs to the power MOSFET.
[ ] Crystal oscilator and pick matching caps.
[ ] Power regulator.
[ ] USB-C connector maybe?
[ ] Debugging connection with the trace line.
[ ] 4 half bridge outputs
[ ] 4 voltage inputs for the 4 outputs
[ ] 1 voltage input for VCC
[ ] 2 (preferably 4) current inputs from the 4 outputs
[ ] maybe 1 current input for total current
[ ] SPI connection to main board. Not ~I2C~, it's too slow and consumes too much current. SPI will need
a chip select pin, but allows for parallel devices. SPI has hardware CRC calculation, seems useful.
[ ] Spare I2C connection for the magnetic sensors.
[ ] Status LED.



TODO for next circuit
---------------------

The AO4266E switching parameter uses Rgen = 3ohm, I think this is their recommended gate resistance.
[ ] Measure gate turn on characteristics of existing circuit...
[ ] Maybe switch to a oscillation damping gate resistor of 3ohm.
[x] Definitely need to think about daisy chaining these, maybe use a signal buffer/transciever with an enable pin,
and/or mosfet switch for the chip power on. I2C to master board is probably the best and easiest way.
[ ] Oopsies I flipped the sign of the V phase current measurement lines.
[ ] The boost gate should maybe be connected to the driver side of the shunt resistor. At the moment we're measuring current used to charge the high gate.
[ ] There's a summing option for the current measurments. Try outputing U, V, U+V, U+V+W using the 4th spare output.
[ ] ! Think carefully about the shunt resistor placement, there's big interference going on if we measure during high side. Timing it around ground as
common mode voltage seems to fix it. Is that the only way to measure all current going through the motor coils?
[ ] ! Measure voltage of the motor coils!
[ ] There's a conflict between i2c and timer 3 channel 2. Choose a different GPIO for the red LED.
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

[ ] Double sided board.
[ ] Think about connectors better, drop USB (the esp32 will have USB-C).
[ ] Drop voltage regulator and rely on master board for 3.3V; filter it though.
[ ] Look for thinner IDC connector, and definitely SMD version for easier routing, use of space behind connector.
[ ] Add voltage probes for the phase connectors so we can measure voltage both during driving and open connections.
We need to measure driving voltage to make sure our MOSFETS are turned on and giving us the voltage we command!
[ ] Use Kelvin sense resistors (or pad hack) for the current measurement to avoid the unknown solder resistance.
https://www.analog.com/media/en/analog-dialogue/volume-46/number-2/articles/optimize-high-current-sensing-accuracy.pdf
[x] Don't drop the hall sensors, yet!
[ ] The big problem with our measurements occurs when we switch from continuous 0 output from the low mosfet to mixed
output with the high mosfet. Any amount of mixed output seems to add an offset to our current measurement. Might be
worth putting the current sense only on the low mosfet branch.
[ ] Choose a different mosfet driver to lower the minimum voltage under 5V.
[ ] Make it work with steppers...




Notes
-----

High voltage limit set by bridge driver (FD6288T) at 20V, with absolute maximum at 25V.
The INA4181 current sense amplifier also works up to 26V on the input.

Continuous current limit set by 1.5A FDC connector from motor to board. Limit is fixed by cable 
that is attached to the motor, can probably be exceeded up to 5A (TODO test connector current!).

Instant current limit is 10A; overpowered MOSFETs can do 60V, 11A with 13.5mΩ resistance. We can
place the STM32 in close proximity to the motor so we can use the onboard temperature sensor to
indirectly measure motor temperature, as well as onboard mosfet temperature (hopefully low). 

Things to think about:
[v] How to control motor? USB for least additional components, I2C for good ol' reliable.
	* CAN bus is fast and reliable but its harder to find IC in stock, annoyingly only JLCPCB option works on 5V.
	* USB is useful to also have a programming interface to the STM32. Potential path for convenient interface,
	will require a multiplexing IC and multile soldered male plugs on master board.
	* SPI is very fast but requires separate line for each motor, will have to multiplex on the master board.
	* I2C is ok fast and ok reliable, simplest to implement, only requires the connector, could easily add 3.3V
	power through a 4 wire plug + 2 wire VCC power on separate connector.
... Turns out the pre-loaded bootloader only works with UART. We also need the boot0 pin and power.

[v] Do we want logic voltage supply lines or overpowered DC-DC converter?
Yes, want the controller to supply 3.3V. Controller should be able to reset boards to bring them into boot mode.
