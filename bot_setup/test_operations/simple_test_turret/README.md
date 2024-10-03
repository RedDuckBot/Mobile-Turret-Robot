Basic test only utilizes pigpio library for testing Stroams 1 degree of freedom turret. Run the executable ./testTurret in build directory. Moreover, this test
is simple since it makes use of gpioInitialize() for direct hardware control, meaning
only one program can have control over the GPIO hardware, so no connection is made
to pigpiod daemon.
