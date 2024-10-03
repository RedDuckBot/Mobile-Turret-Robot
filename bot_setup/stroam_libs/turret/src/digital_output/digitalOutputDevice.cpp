#include "digitalOutputDevice.hpp"

namespace outputDevice
{
    DigitalOutputDevice::~DigitalOutputDevice() 
    {
        fmt::print("Closing GPIO pin {}\n",device_GPIO_pin_);
        off();
    }

    DigitalOutputDevice::DigitalOutputDevice(unsigned int gpioPin)
    {
        this -> device_GPIO_pin_ = gpioPin;
        set_mode(getGPIOHandle(), gpioPin, PI_OUTPUT);
        off();
    }

    void DigitalOutputDevice::on()
    {
        gpio_write(getGPIOHandle(), device_GPIO_pin_, 1);
    }

    void DigitalOutputDevice::off()
    {
        gpio_write(getGPIOHandle(), device_GPIO_pin_, 0);
    }
}
