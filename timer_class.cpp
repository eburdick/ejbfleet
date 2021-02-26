#include <arduino.h>
#include "timer_class.hh"

        Timer::Timer (void)
        {
            endTimeExists = false;
            endTime = 0; // for testing a timer that hasn't been started yet.
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
            /*
             * If the timer has been started with a time, then an end time exists and
             * we test whether it has been reached. If an end time does not exist, then
             * the timer has never been given a timeout period, so testing it really 
             * makes no sense. We return true as if the timer is between uses.
             */
            if (endTimeExists)
            {
                if (millis() > endTime)
                    return (true);
                else
                    return (false);
            }
            else
            {
                return (true);
            }
        }

        long Timer::TimeSinceStart(void)
        {
            /*
             * This is just a member function to check how much time has passed since the
             * the timer was last started, with or without an end time.
             */
            return (millis() - startTime);
        }
