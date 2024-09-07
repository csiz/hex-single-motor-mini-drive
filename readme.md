Mini FOC driver
===============

Current control, FOC driver board for 3 phase BLDC motors with hall effect sensor feedback.

Maximum recommended limits: 20V, 3A(continuous), 10A(instant).

TODO for next circuit
---------------------

The AO4266E switching parameter uses Rgen = 3ohm, I think this is their recommended gate resistance.
[ ] Measure gate turn on characteristics of existing circuit...
[ ] Maybe switch to a oscillation damping gate resistor of 3ohm.

Need to implement in the circuit
--------------------------------

[v] Measure power line voltage.
[v] Pick voltage boost capacitor and resistor pair.
[v] Pick gate oscillation damping resistor for the inputs to the power MOSFET.
[v] Crystal oscilator. Need to improve motor control timing and UART, USB. Also, we need it to get to 72MHz!
	[v] Pick matching caps.
	[v] Connect and verify!
[v] Get micro USB for the convenience. Will also need to add 3V3 power regulator and ferrite bead for input filtering.
	[v] Special note for STM32F103 line to add pull up resistor for D+ line.
	[v] Connect and verify!
[v] Debugging connection with the trace line.
[v] UART connection to main board.
[v] I2C header? And some power line holes as alternative connector to be friendly to hobbyist use. Also for the magnetic encoder.
[v] Add a status LED.
[v] How to connect leftover pins?




Notes
-----

High voltage limit set by bridge driver (FD6288T) at 20V, with absolute maximum at 25V.

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
