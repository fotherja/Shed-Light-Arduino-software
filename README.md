# Shed-Light-Arduino-software
Arduino code for solar powered shed light system

The arduino runs a basic MPPT to charge a 6S 5.8Ah LiPo battery from a 10W solar panel. 
If the battery voltage falls too low, a MOSFET cuts power from being drawn.
If the battery is fully charged, the arduino goes into a very low power sleep mode.

The MPPT works by operating a boost converter at a PWM that maintains a 17v voltage at the solar panel. This is roughly the MPP.

It's pretty simple!
