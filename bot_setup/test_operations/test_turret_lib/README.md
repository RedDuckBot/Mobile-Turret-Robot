# Test Turret Library

This test project consumes the target Turret::turret from Stroam's turret library,
and checks for the working operations of turret interface.

## Dependencies 
- fmt
- pigpio
- Stroam's turret library 
- Stroam's device library

## Before Compilation
Create symbolic links for both Stroam's libraries device and turret: 
- TurretTargets.cmake
- DeviceTargets.cmake

Note: both export target files are found in the libraries build/cmake directory after they are built
