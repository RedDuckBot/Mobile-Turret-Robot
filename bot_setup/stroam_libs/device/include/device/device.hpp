#ifndef DEVICE
#define DEVICE

#include <pigpio.h>
#include <fmt/core.h>

namespace device
{
    class Device
    {
        public:
            Device();
            virtual ~Device();

        private:
            static int deviceCount; //Number of GPIO pins in use when using pigpio
                                    //library
    };
}
#endif