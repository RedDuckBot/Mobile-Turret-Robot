#include "../include/device/device.hpp"

namespace device
{
    int Device::deviceCount = 0;
    int Device::gpioHandle = -1;

    Device::Device()
    {
        if (deviceCount == 0)
        {
            //Locally, connect to pigpio daemon (pigpiod)
            gpioHandle = pigpio_start(NULL,NULL);
            if (gpioHandle < 0)
            {
                throw std::runtime_error(fmt::format("Failed to connect to pigpiod\n"));
            }
        }
        deviceCount++;
    }

    Device::~Device()
    {
        deviceCount--;
        if (deviceCount == 0)
        {
            pigpio_stop(gpioHandle);
            fmt::print("Closing pigpiod daemon.\n");
        }
    }

    int Device::getGPIOHandle() const
    {
        return gpioHandle;
    }
}