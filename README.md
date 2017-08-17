# Sensor2USB
Converts industrial sensors into keystrokes

Ever wanted to interface a couple of sensors to a PC without getting a whole DAQ board involved? Probably not, but here's a solution if you do :)

Sensor2USB is a simple V-USB based project that converts industrial sensors into keystrokes by identifying as a USB HID device.

Supports up to two sensors, but each board can be given a seperate address (up to 5 total) by holding down the button.

Supports PNP/NPN sensors using an opto-coupler, just reverse the wiring depending on the sensor!

Provides a 0.1A 24V boost regulator to power most sensors, works with switches as well.
