#include "../include/turret/turret.hpp"

namespace turret
{
    Turret::Turret(
        //Pins defined as GPIO pins
        unsigned int laser_pin, 
        unsigned int shooting_motors_pin, 
        unsigned int base_servo_pin, 
        unsigned int pusherRod_pin): 
            laser_(laser_pin),
            shootingMotors_(shooting_motors_pin),
            baseServo_(base_servo_pin, MIN_BASE_SERVO, MAX_BASE_SERVO),
            pusherRodServo_(pusherRod_pin, MIN_PUSHER_SERVO, MAX_PUSHER_SERVO)
        {

        }

    void Turret::laser_on()
    {
        laser_.on();
    }

    void Turret::laser_off()
    {
        laser_.off();
    }

    void Turret::move_to_pos(int angle)
    {
        baseServo_.rotate_to_pos(angle);
    }

    void Turret::fire()
    {

    }
}
