# Test Turret Library

This test project consumes the target Turret::turret from Stroam's turret library,
and checks for the working operations of turret interface.

## Dependencies 
- fmt
- pigpio_if2
- Stroam's turret library 
- Stroam's device library

## Before Build & Compilation
In the root directory, Create symbolic links for both Stroam's libraries device and turret: 
- TurretTargets.cmake
- DeviceTargets.cmake

Note: both export target files are found in the libraries build/cmake directory after they are built

## Before running Test
Start pigpiod daemon:
```
sudo pigpiod
sudo killall pigpiod #After done running test close pigpiod daemon
```