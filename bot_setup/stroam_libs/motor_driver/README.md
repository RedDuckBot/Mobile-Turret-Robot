# Motor Driver

C++ library for representing a motor controller. Stroam is using an L298N motor controller. The export target file in ./build/cmake, after build sys. & compilation, is called MotorDriverTargets.cmake .

## Dependencies
- [`fmt`](https://github.com/fmtlib/fmt) : C++ library for formatting
- [`device`](../device/) : C++ library from Stroam's libraries 
- [`turret`](../turret/) : C++ library from Stroam's libraries; MotorDriver uses DigitalOutputDevice from this lib 

## Pinouts
 |GPIO Pin| L298N Pin | Motors | 
| --| --| --|
|23| IN1 | Right side | 
|24| IN2 | Right side |
|25| ENA | Right side |
|17| IN3 | Left side |
|27| IN4 | Left side |
|22| ENB | Left side |

Table above represents the default pinouts.