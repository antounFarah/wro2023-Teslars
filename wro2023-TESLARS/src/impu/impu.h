#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

class impu
{
private:
    #define OUTPUT_READABLE_YAWPITCHROLL
    #define INTERRUPT_PIN 2
    bool blinkState = false;

    bool dmpReady = false; 
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    Quaternion q; 
    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;    

    float euler[3];
    float ypr[3];
    uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

    volatile bool mpuInterrupt = false;
    int offx = 0, offy = 0;

public:
    impu(int& setx, int& sety);
    ~impu();
    void init();
    void gessoffsets();
    void getyaw(float& yaw);
};

inline impu::impu(int& setx, int& sety) {
    
    offx = setx;
    offy = sety;

 }

impu::~impu() { }

inline void impu::init() 
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
}

inline void impu::gessoffsets() 
{ 
    mpu.setXGyroOffset(offx);
    mpu.setYGyroOffset(offy);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(-10);
}

inline void impu::getyaw(float& yaw) 
{ 
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/M_PI;
    }
}
