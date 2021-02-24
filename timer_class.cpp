#include <arduino.h>
#include "timer_class.hh"

        Timer::Timer (void)
        {

        }
        void Timer::Start(void)
        {
            startTime = millis();
            endTimeExists = false;
        }
        void Timer::Start(int delay)
        {
            startTime = millis();
            endTime = startTime + delay;
            endTimeExists = true;
        }
        boolean Timer::Test(void)
        {
            if (endTimeExists)
            {
                if (endTime < millis())
                    return (true);
                else
                    return (false);
            }
            else
            {
                return (false);
            }
        }

        long Timer::TimeSinceStart(void)
        {
            return (millis() - startTime);
        }
