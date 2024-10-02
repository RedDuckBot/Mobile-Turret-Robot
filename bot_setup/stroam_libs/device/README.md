# Device

C++ library containing a parent class called 'Device' for any C++ class in Stroam's 
library that utilzes the C library pigpio. This class handles initialization for
pigpio daemon and closing GPIO pins used.

## Dependencies
- [pigpio](https://github.com/joan2937/pigpio) C library for GPIO control

## Stroam's Devices
- DigitalOutputDevice --> turret library
- Servo --> turret library