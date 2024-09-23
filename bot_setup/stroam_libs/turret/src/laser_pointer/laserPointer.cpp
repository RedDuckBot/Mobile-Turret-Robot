#include "laserPointer.hpp"
#include <fmt/core.h>

namespace laser
{
    LaserPointer::LaserPointer(uint32_t gpioPin):
        laserPin_(gpioPin)
    {
        off();
    }

    void LaserPointer::on()
    {
        laserPin_.setOutput(true);
    }

    void LaserPointer::off()
    {
        laserPin_.setOutput(false);
    }
}
