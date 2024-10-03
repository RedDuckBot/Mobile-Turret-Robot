#include "turret/turret.hpp"
#include <chrono>
#include <thread>

int main()
{
    auto turret = new turret::Turret();
    turret->laser_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    turret->laser_off();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    turret->laser_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    delete turret;
    return 0;
}
