#include "../include/turret/turret.hpp"

namespace turret
{
    void Turret::laser_on()
    {
        laser.on();
    }

    void Turret::laser_off()
    {
        laser.off();
    }
}
