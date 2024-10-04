#include "../include/turret/turret.hpp"

namespace turret
{
    Turret::Turret(
        //Pins defined as GPIO pins
        unsigned int laser_pin, 
        unsigned int shooting_motors_pin, 
        unsigned int base_servo_pin, 
        unsigned int pusherRod_pin): 
            laser_(laser_pin, "laser"),
            shootingMotors_(shooting_motors_pin, "shooting motors"),
            baseServo_(base_servo_pin, MIN_BASE_SERVO, MAX_BASE_SERVO, 
                90, "base servo"),
            pusherRodServo_(pusherRod_pin, MIN_PUSHER_SERVO, MAX_PUSHER_SERVO,
                180, "pusher rod servo")
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

    void Turret::move_rod_forward()
    {
        pusherRodServo_.rotate_to_pos(0);
    }

    void Turret::move_rod_backward()
    {
        pusherRodServo_.rotate_to_pos(180);
    }

    void Turret::move_to_pos(int angle)
    {
        baseServo_.rotate_to_pos(angle);
    }

    void Turret::fire()
    {
        shootingMotors_.on();
        //std::this_thread::sleep_for(std::chrono::milliseconds(4500));
        sleep(4.5); //4.5 seconds
        pusherRodServo_.rotate_to_pos(0);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        sleep(2);
        shootingMotors_.off();
        pusherRodServo_.rotate_to_pos(180);
    }
}
