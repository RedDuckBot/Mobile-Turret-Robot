#ifndef TURRET
#define TURRET

#include "digitalOutputDevice.hpp"
#include "servo.hpp"

//Define pulse width ranges for base servo and pusher rod servo
#define MAX_BASE_SERVO 2500
#define MIN_BASE_SERVO 550
#define MAX_PUSHER_SERVO 2400
#define MIN_PUSHER_SERVO 600 

namespace turret
{
    class Turret
    {
        public:
            Turret();
            void move_to_pos(int angle);
            void laser_on();
            void laser_off();
            void fire();

        private:
            outputDevice::DigitalOutputDevice laser_;
            outputDevice::DigitalOutputDevice shootingMotors_;
            servo::Servo baseServo_;
            servo::Servo pusherRodServo_;
    };
}
#endif
