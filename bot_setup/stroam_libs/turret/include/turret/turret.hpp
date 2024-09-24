#ifndef TURRET
#define TURRET

#include "laserPointer.hpp"
#include "servo.hpp"
#include "shootingMotors.hpp"

namespace turret
{
    class Turret
    {
        public:
            Turret();
            void rotate();
            void laser_on();
            void laser_off();
            void fire();

        private:
            laser::LaserPointer laser;
            servo::Servo baseServo;
            servo::Servo pusherRodServo;



    };
}
#endif
