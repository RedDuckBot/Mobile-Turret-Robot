#include "components/LaserPointer.hpp"
#include <fmt/core.h>

namespace Components
{
    LaserPointer::LaserPointer(uint32_t gpioPin):
        laserPin_(gpioPin)
    {
        fmt::print("Laser ready\n");
    }

    void LaserPointer::on()
    {
        fmt::print("ON\n");

    }

    void LaserPointer::off()
    {

        fmt::print("ON\n");
    }
}
