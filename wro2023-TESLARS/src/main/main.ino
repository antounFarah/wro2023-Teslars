#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo steering;

MPU6050 mpu;

const int trigpin[] = {4, 5, 6, 7};
const int echopin[] = {2, 3, 18, 19};

int i;

int yaw;

volatile int measured[] = {0, 0, 0, 0};
volatile long duration[4];
volatile long timer[4];
volatile double distance[4];

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
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

volatile bool mpuInterrupt = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    Serial.begin(115200);
    while (!Serial)
        ;
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Serial.println(F("Testing device connections..."));
    //  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    // offset
    mpu.setXGyroOffset(41);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(-10);

    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    for (i = 0; i < 4; i++)
    {
        pinMode(trigpin[i], OUTPUT);
        pinMode(echopin[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(echopin[0]), read_echo_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[1]), read_echo_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[2]), read_echo_3, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[3]), read_echo_4, FALLING);

    servo1.attach(8);
    servo2.attach(9);
    servo3.attach(10);
    servo4.attach(11);
}
void loop()
{
    steering.write(pid(distance[2], 40, 1, 0.0002, 0.5));
    if (!dmpReady)
        return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("\t");
        yaw = ypr[0] * 180 / M_PI;
        
        servo1.write(yaw);
        servo2.write(yaw);
        servo3.write(yaw);
        servo4.write(yaw); 

        Serial.println(yaw);
    }
}

void dmpDataReady()
{
    mpuInterrupt = true;
}

void send_trig(int x)
{
    measured[x] = 1;
    digitalWrite(trigpin[x], LOW);
    delayMicroseconds(2);
    digitalWrite(trigpin[x], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin[x], LOW);
    delayMicroseconds(2);
    while (digitalRead(echopin[x]) == LOW) {}
    timer[x] = micros();
}

void read_echo_1()
{
    duration[0] = micros() - timer[0];
    distance[0] = (duration[0] * 0.034 / 2);
    measured[0] = 0;
}
void read_echo_2()
{
    duration[1] = micros() - timer[1];
    distance[1] = (duration[1] * 0.034 / 2);
    measured[1] = 0;
}
void read_echo_3()
{
    duration[2] = micros() - timer[2];
    distance[2] = (duration[2] * 0.034 / 2);
    measured[2] = 0;
}
void read_echo_4()
{
    duration[3] = micros() - timer[3];
    distance[3] = (duration[3] * 0.034 / 2);
    measured[3] = 0;
}

float pid(float ultrasonic_value, float goal, float kp, float ki = 0, float kd = 0) {
    float angle, propotional, integral = 0, derivediv, error, last_error = 0, i = 0 ;
    propotional = (ultrasonic_value - goal) * kp;
    error = ultrasonic_value - goal;
    i = integral + error;
    integral = i * ki;
    derivediv = (error - last_error) * kd;

    angle = propotional + integral + derivediv;
    last_error = error;
    return angle;
}
