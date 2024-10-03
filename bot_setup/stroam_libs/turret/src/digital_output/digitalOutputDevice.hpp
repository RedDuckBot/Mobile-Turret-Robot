#ifndef OUTPUTDEVICE
#define OUTPUTDEVICE

#include <pigpio.h>
#include <fmt/core.h>
#include "../../../device/include/device/device.hpp"

namespace outputDevice
{
    class DigitalOutputDevice : public device::Device
    {
    	public:
            DigitalOutputDevice(unsigned int gpioPin);

            void on();
            void off();
            virtual ~DigitalOutputDevice();

        private:
            unsigned int device_GPIO_pin_; 
    };
}
#endif 
