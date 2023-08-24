// Class for rotary encoder

#ifndef RotaryEncoder_h
#define RotaryEncoder_h
#endif

#include "Arduino.h"

class RotaryEncoder
{
    private:
        // Circular buffer
        // const int bufferSize = 10;
        #define bufferSize 10
        double buffer[bufferSize];
        int bufferIndex = 0;

        // Velocity timer
        unsigned long oldTime = 0;
        unsigned long newTime = 0;
        unsigned long velocityTimer = 0;

        double stepToRad(int step)
        {
            // Convert step to radians
            double rad = double(step) * 2 * PI / 4096 - PI;

            // Return radians
            return rad;
        };

    public:
        // Constructor
        RotaryEncoder()
        {
            // Constructor code goes here
        };

        // Update
        void update(int step)
        {
            // Add value to buffer
            buffer[bufferIndex] = stepToRad(step);

            // Increment buffer index
            bufferIndex++;

            // Check if buffer index is at the end
            if (bufferIndex == bufferSize)
            {
                // Reset buffer index
                bufferIndex = 0;
            }

            // Update velocity timer
            oldTime = newTime;
            newTime = micros();
            velocityTimer = newTime - oldTime;
        };

        // Get value from circular buffer, index 0 is the most recent value, -1 is the second most recent, etc.
        // Added -1 to get most recent value, since bufferIndex points to the next index to be written
        double getPos(int index = 0)
        {
            int buffer_index_temp = (bufferSize + bufferIndex -1 + index) % bufferSize;
            return buffer[buffer_index_temp];
        };

        // Get velocity, calculate from 2 step method, returns radians per second
        // Use velocityTimer to calculate velocity, Use micros() to get current time
        double getVel()
        {
            // Calculate time difference
            double timeDifference = velocityTimer / 1000000.0;

            // Calculate velocity
            double average_velocity_n = 0;
            int n = 1; // moving average of n values
            for (auto i = 0; i < n; i++) {
                // Current pos minus previous pos
                auto delta_pos = getPos(-i)-getPos(-i-1);
                average_velocity_n += atan2(sin(delta_pos), cos(delta_pos)) / timeDifference;
            }
            double velocity = average_velocity_n / n;

            // Print last 2 values
            // Serial.print("new: ");
            // Serial.print(getPos(0));
            // Serial.print(" old: ");
            // Serial.print(getPos(-1));
            // Serial.print(" diff: ");
            // Serial.print(timeDifference);
            // Serial.print(" vel: ");

            // Return velocity
            return velocity;
        };

};


