# Turret 

A C++ library representing an interface for Stroam's turret. 

## Dependencies
- [`pigpio_if2`](https://github.com/joan2937/pigpio) : C library for GPIO control using pigpiod daemon
- [`fmt`](https://github.com/fmtlib/fmt) : C++ library for formatting
- [`device`](../device/) : C++ library from Stroam's libraries; contains a 
parent class, called Device, that handles initialization and deconstruction for devices found in this library

## Pinouts
|Device| GPIO Pin |
| --| --|
|Laser| 26 |
|Base Servo| 13 |
|Pusher Rod Servo| 12 |
|Shooting Motors| 16 |

Table above represents the default pinouts.

## Before Build sys. & Compilation 
To handle the 'device' dependency create a symbolic link to the export target file found in Stroam's device library. Name this file as 'DeviceTargets.cmake'.
Note: this file is found in the cmake directory of build in device library after compiling it.