#include "../include/turret/turret.hpp"

namespace turret
{
    Turret::Turret(unsigned int laser_gpio_pin=26, unsigned int 
        shooting_gpio_pin=16, unsigned int servo_gpio_pin=13, unsigned int
        pusherRod_gpio_pin=12): 
        laser_(laser_gpio_pin),
        shootingMotors_(shooting_gpio_pin),
        baseServo_(servo_gpio_pin, MIN_BASE_SERVO, MAX_BASE_SERVO),
        pusherRodServo_(pusherRod_gpio_pin, MIN_PUSHER_SERVO, MAX_PUSHER_SERVO)
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
}
