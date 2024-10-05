# Test Motor Driver Library

This project consumes the target MotorDriver::motorDriver from Stroam's motor driver library and checks for the working operations of the motor driver. 

##Dependencies
- fmt
- pigpio_if2
- Stroam's motor driver library
- Stroam's device library
- Stroam's turret library (MotorDriver uses DigitalOutputDevices from turret lib)

## Before Build sys. & Compilation
In the root directory of this test project, create two symbolic links:
- DeviceTargets.cmake (from device library)
- MotorDriverTargets.cmake (from motor driver library)

Note: both export target files are found in the libraries build/cmake directory after building & compiling them

## Before running Test

Start pigpiod daemon:

```
sudo pigpiod
sudo killall pigpiod #After done running test close pigpiod daemon
```