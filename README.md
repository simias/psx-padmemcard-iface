# ATmega4809-based interface for PlayStation Memory Cards and Controllers

This repository contains the schematics and controller code to create a simple
UART-based interface to PlayStation Memory Card and Controllers.

The design uses a simple ATmega4809 microcontroller running at 4MHz.

The PlayStation Pad/MemCard connector uses both 3.5V (for all communication
signals) and 7.5V (mainly used for DualShock rumble motors and other
power-hungry features). I think that most devices will work perfectly fine with
only the 3.5V input, but in order to be as close as possible to the real
hardware I power my board with a 7.5V that I can wire to the relevant pin of the
Pad/MemCard port, and I use a cheap off-the-shelf DC-DC module to convert that
7.5V to 3.5V which I use to power the controller.

I also send the 5V from the UART-to-USB and UPDI inputs to the DC-DC input
(through diodes) to allow powering the board from USB, but in this case this
same 5V will be sent to the Pad/MemCards instead of a true 7.5V.

A single LED is pulsed with PWM to show activity.
