#include "components/LaserPointer.hpp"

using Components::LaserPointer;

int main()
{
    int laserPinGPIO = 26;
    LaserPointer laser(laserPinGPIO);   

    laser.on();
    laser.off();

    return 0;
}
