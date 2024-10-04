#ifndef SERVO
#define SERVO

#include <pigpiod_if2.h>
#include <string>
#include <unistd.h>
#include "../../../device/include/device/device.hpp"

using device::Device;

namespace stroams_servo
{
    class Servo : public Device
    {
        public:
            Servo(unsigned int servoPin, unsigned int min_pulse_width, 
                unsigned int max_pulse_width, int starting_angle=0, 
                const std::string& servoName="");
            virtual ~Servo(); 

            //Angle between 0 and 180 deg
            void rotate_to_pos(int angle, bool stop_servo=false); 

        private:
            unsigned int servoPin_; 
            int starting_angle_; //Degrees
            unsigned int min_pulse_width_;
            unsigned int max_pulse_width_;

            //utility function for rotate function
            unsigned int get_pwm_value_from_angle(int angle);
    };
}
#endif
