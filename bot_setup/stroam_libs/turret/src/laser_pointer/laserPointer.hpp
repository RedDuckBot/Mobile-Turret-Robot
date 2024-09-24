#ifndef LASER
#define LASER

#include <pigpio.h>

namespace laser
{
    class LaserPointer
    {
    	public:
            LaserPointer();

            void on();
            void off();

        private:
        const unsigned int laserPin_ = 26;
    };
}
#endif 
