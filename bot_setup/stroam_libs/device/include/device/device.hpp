#ifndef DEVICE
#define DEVICE

#include <pigpiod_if2.h>
#include <fmt/core.h>

namespace device
{
    class Device
    {
        public:
            Device();
            virtual ~Device();

        protected:
            int getGPIOHandle() const;

        private:
            //Number of GPIO pins in use when using pigpio library 
            static int deviceCount; 
            static int gpioHandle;
    };
}
#endif