#include "../include/device/device.hpp"

namespace device
{
    int Device::deviceCount = 0;

    Device::Device()
    {
        if (deviceCount == 0)
        {
            if (gpioInitialise() < 0)
            {
                throw std::runtime_error(fmt::format("Failed to initialize GPIO"));
            }
        }
        deviceCount++;
    }

    Device::~Device()
    {
        deviceCount--;
        if (deviceCount == 0)
        {
            gpioTerminate();
        }
    }
}