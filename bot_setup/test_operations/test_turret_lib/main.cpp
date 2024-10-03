#include "turret/turret.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>

using turret::Turret;

void test_laser(const std::unique_ptr<Turret>& turret)
{
    turret->laser_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    turret->laser_off();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    turret->laser_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    return;
}


int main()
{
    std::unique_ptr<Turret> turret = std::make_unique<Turret>();
    test_laser(turret);
    return 0;
}

