#include "laserPointer.hpp"

namespace laser
{
    LaserPointer::LaserPointer()
    {
        gpioSetMode(laserPin_, PI_OUTPUT);
        off();
    }

    void LaserPointer::on()
    {
        gpioWrite(laserPin_, 1);
    }

    void LaserPointer::off()
    {
        gpioWrite(laserPin_, 0);
    }
}
