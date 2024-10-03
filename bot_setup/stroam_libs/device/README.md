# Device

C++ library containing a parent class called 'Device' for any C++ class in Stroam's 
library that utilzes the C library pigpio. This class handles initialization for
pigpiod daemon and closing it after all devices have been closed.

## Dependencies
- [pigpio_if2](https://github.com/joan2937/pigpio) C library for GPIO control

## Stroam's Device Classes
- DigitalOutputDevice --> turret library
- Servo --> turret library