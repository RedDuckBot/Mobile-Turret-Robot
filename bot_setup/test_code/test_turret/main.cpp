#include <iostream>
#include <fmt/core.h>
#include <pigpio.h>
#include <chrono>
#include <thread>

//Define GPIO pins
int motorPin = 16; //Connected to Relay pin 
int baseServoPin = 13;
int laserPin = 26;
//int pusherPin;
bool motors_on = false;
bool laser_on = false;

//Pulse width range for servo MG99GR 
#define MIN 550
#define MAX 2500

using namespace fmt;

bool check_gpio_initialized();
void setup_gpio_pins();
void run_turret_test();
void display_command_prompts();
void rotate_turret(int, int, int, int);
void close_gpio();
int get_pwm_value_from_angle(int);

int main()
{
    bool gpio_init_pass;

    gpio_init_pass = check_gpio_initialized(); 

    if ( gpio_init_pass == true)
    {
        setup_gpio_pins();
        run_turret_test();
    }

    return 0;
}

void setup_gpio_pins()
{
    gpioServo(baseServoPin, MIN);
    gpioSetMode(motorPin, PI_OUTPUT);
    gpioWrite(motorPin, 0);
    gpioSetMode(laserPin, PI_OUTPUT);
    gpioWrite(laserPin, 0);
}

bool check_gpio_initialized()
{
    if (gpioInitialise() < 0)
    {
        std::cerr << "Failed to initialize GPIO" << std::endl;
        return false;
    }

    return true;
}

void run_turret_test()
{
    bool running = true;
    int command;

    display_command_prompts();

    while (running)
    {
        print("Enter command: ");
        std::cin >> command;

        switch(command)
        {
            case 1: //toggle shooting motors
                if (motors_on)
                {
                    print("Motors turned OFF\n");
                    motors_on = false;
                    gpioWrite(motorPin,0);
                }
                else
                {
                    print("Motors turned ON\n");
                    motors_on = true;
                    gpioWrite(motorPin,1);
                }
                break;
            case 2: //toggle laser
                if (laser_on)
                {
                    print("Laser turned OFF\n");
                    laser_on = false;
                    gpioWrite(laserPin,0);
                }
                else
                {
                    print("Laser turned ON\n");
                    laser_on = true;
                    gpioWrite(laserPin,1);
                }
                break;
            case 3: //swing turret
                print("Rotating Turret\n");
                rotate_turret(0,180,3,175);
		break;
            case 4: //Exit Program
                print("Exiting Program\n");
                close_gpio();
                running = false;
                break;
            default:
                print("Not a valid command\n");
        }
    }
}

void display_command_prompts()
{
    print("Available Commands as integers:\n");
    print("\t1 --> Toggle shooting Motors\n");
    print("\t2 --> Toggle Laser\n");
    print("\t3 --> Swing Turret\n");
    print("\t4 --> Exit Testing\n\n\n");

}

/*
  Purpose: Rotate turret from min position to max position
  Parameters: startAngle <-- begining angle of rotation
              endAngle <-- end of rotation
              pause <-- The number of miliseconds to pause before next rotation
*/
void rotate_turret(int startAngle, int endAngle, int angleIncrement, int pause)
{
    int current_PWM;

    //Rotate forward
    for (int currAngle = startAngle; currAngle <= endAngle; currAngle += 
        angleIncrement)
    {
        current_PWM = get_pwm_value_from_angle(currAngle);
        gpioServo(baseServoPin, current_PWM);
        std::this_thread::sleep_for(std::chrono::milliseconds(pause));
    }

    //Rotate backward
    for (int currAngle = endAngle; currAngle >= startAngle; currAngle -= 
        angleIncrement)
    {
        current_PWM = get_pwm_value_from_angle(currAngle);
        gpioServo(baseServoPin, current_PWM);
        std::this_thread::sleep_for(std::chrono::milliseconds(pause));
    }
}

int get_pwm_value_from_angle(int angle)
{
    double x;
    int pwm_value;
    double pwm_range = MAX - MIN;

    x = angle / 180.0;
    x = pwm_range * x;
    pwm_value = (int) (MIN + x);

    if (pwm_value > MAX)
    {
        pwm_value = MAX;
    }

    if (pwm_value < MIN)
    {
        pwm_value = MIN;
    }

    return pwm_value;
}

void close_gpio()
{
    gpioWrite(motorPin,0);
    gpioWrite(laserPin,0);
    gpioTerminate();
}
