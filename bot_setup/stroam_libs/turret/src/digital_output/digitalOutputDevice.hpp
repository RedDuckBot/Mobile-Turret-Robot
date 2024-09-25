#ifndef DIGITALOUTPUTDEVICE
#define DIGITALOUTPUTDEVICE

#include <pigpio.h>

namespace outputDevice
{
    class DigitalOutputDevice
    {
    	public:
            DigitalOutputDevice(unsigned int gpioPin);

            void on();
            void off();

        private:
        const unsigned int device_GPIO_pin_; 
    };
}
#endif 
