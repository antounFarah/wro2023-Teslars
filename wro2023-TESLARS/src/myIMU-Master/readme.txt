this Library used for get yaw angle of the MPU6050 module.
it is an easy-to-use interface for robotic solutions.

how to use this library:
    -   first you need to include it and define object by write:
            #include "myIMU.h"
            #include "SampleConfig.h"

            SampleConfig config;
            myIMU& mpu = config.getMPU();

    -   and then initialize the MPU6050 module:
            mpu.init();
    -   finally you can now use the getyaw() method:
            int yaw;
            mpu.getyaw(yaw);

