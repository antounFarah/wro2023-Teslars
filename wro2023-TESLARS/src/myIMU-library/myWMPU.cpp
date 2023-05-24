#include "myWMPU.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
namespace nsmpu {
    MPU6050 mpu;
}


void myWMPU::init()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    nsmpu::mpu.initialize();
    nsmpu::mpu.testConnection();
    uint8_t devStatus;

    devStatus = nsmpu::mpu.dmpInitialize();
    gessoffsets();
    if (devStatus == 0) {
        nsmpu::mpu.setDMPEnabled(true);

    }
}

void myWMPU::getyaw(float& y)
{
    uint8_t fifoBuffer[64];
    Quaternion q;
    if (nsmpu::mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        nsmpu::mpu.dmpGetQuaternion(&q, fifoBuffer);

        float q0 = q.w;
        float q1 = q.x;
        float q2 = q.y;
        float q3 = q.z;

        float yr = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 - q1 * q1 + q0 * q0);
        
        y = yr * 180 / M_PI;
    }

}

void myWMPU::gessoffsets()
{
    nsmpu::mpu.setXGyroOffset(220);
    nsmpu::mpu.setYGyroOffset(76);
    nsmpu::mpu.setZGyroOffset(-85);
    nsmpu::mpu.setZAccelOffset(1788);
}
