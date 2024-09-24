#ifndef SERVO
#define SERVO

#include <pigpio.h>

namespace servo
{
    class Servo
    {
        public:
            Servo(unsigned int servoPin, unsigned int min_pulse_width, 
                unsigned int max_pulse_width);
            

            void rotate_to_pos(int angle); //Angle between 0 and 180 deg

        private:
            unsigned int servoPin_; 
            unsigned int min_pulse_width_;
            unsigned int max_pulse_width_;

            //utility function for rotate function
            unsigned int get_pwm_value_from_angle(int angle);

    };
}
#endif
