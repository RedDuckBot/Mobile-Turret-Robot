#include "turret/turret.hpp"
#include <memory>
#include <iostream>
#include <fmt/core.h>

using turret::Turret;
using namespace fmt;

void display_command_prompts()
{
    print("Available Commands as integers:\n");
    print("\t1 --> Fire a foam ball\n");
    print("\t2 --> Turn laser on\n");
    print("\t3 --> Turn laser off\n");
    print("\t4 --> Move pusher rod forward\n");
    print("\t5 --> Move pusher rod backward\n");
    print("\t6 --> Move turret to position [0-180 degrees]\n");
    print("\t7 --> Exit Testing\n\n\n");
}

void run_turret_test(const std::unique_ptr<Turret>& turret)
{
    bool running_test = true;
    int command;
    int angle;

    display_command_prompts();

    while (running_test)
    {
        print("Enter command: ");
        std::cin >> command;

        switch(command)
        {
            case 1: //Fire a bullet!
                print("Firing!\n");
                turret->fire();
                break;
            case 2: //Turn laser on
                print("Turning laser on\n");
                turret->laser_on();
                break;
            case 3: //Turn laser off 
                print("Turning laser off\n");
                turret->laser_off();
		        break;
            case 4:
                print("Moving rod forward\n");
                turret->move_rod_forward();
                break;
            case 5:
                print("Moving rod backward\n");
                turret->move_rod_backward();
                break;
            case 6: // rotate to position
                print("Enter an angle: ");
                std::cin >> angle;
                turret->move_to_pos(angle);
                break;
            case 7: //Exit Program
                print("Exiting Program\n");
                running_test = false;
                break;
            default:
                print("Not a valid command\n");
        }
    }
}

int main()
{
    std::unique_ptr<Turret> turret = std::make_unique<Turret>();

    run_turret_test(turret);
    return 0;
}