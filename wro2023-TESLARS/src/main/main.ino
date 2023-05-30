#include "impu.h"
#include "Pixy2.h"

#include <Servo.h>

#define encoder1 21
#define no_of_holes 20
volatile int counter = 0; 

Pixy2 pixy;

int mx = 41, my = 20;
impu imu(mx, my);


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// used arduino mega pins
/*
4, 5, 6, 7, 18, 19: ultrasonic
8, 9, 10, 11 : ultrasonic servos

*/
const int trigpin[] = { 4, 5, 6, 7 };
const int echopin[] = { 2, 3, 18, 19 };

int i, j;

float yaw;

volatile int measured[] = { 0, 0, 0, 0 };
volatile long duration[4];
volatile long timer[4];
volatile double distance[4];

void setup()
{
    imu.init();
    pixy.init();

    pinMode(encoder1, INPUT);

    for (i = 0; i < 4; i++) {
        pinMode(trigpin[i], OUTPUT);
        pinMode(echopin[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(echopin[0]), read_echo_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[1]), read_echo_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[2]), read_echo_3, FALLING);
    attachInterrupt(digitalPinToInterrupt(echopin[3]), read_echo_4, FALLING);

    attachInterrupt(digitalPinToInterrupt(encoder1), count, RISING);

    servo1.attach(8);
    servo2.attach(9);
    servo3.attach(10);
    servo4.attach(11);
}
void loop()
{

    imu.getyaw(yaw);
    servo1.write(yaw);
    servo2.write(yaw);
    servo3.write(yaw);
    servo4.write(yaw);
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
    while (digitalRead(echopin[x]) == LOW) { }
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

float pid(float ultrasonic_value, float goal, float kp, float ki = 0, float kd = 0)
{
    float angle, propotional, integral = 0, derivediv, error, last_error = 0, i = 0;
    propotional = (ultrasonic_value - goal) * kp;
    error = ultrasonic_value - goal;
    i = integral + error;
    integral = i * ki;
    derivediv = (error - last_error) * kd;

    angle = propotional + integral + derivediv;
    last_error = error;
    return angle;
}
void count(){
    counter += 1;
}
float medianfilter(float values[4]){
    for (i = 0 ; i < 4; i++){
        for (j = 4; j > i; j--){
            if (values[j] > values[i]){
                float storage = values[i];
                values[i] = values[j];
                values[j] = values[i];
            }
        }
    }
    return values[2];
}
void runningavgfilter(float last_read, float new_read){
    new_read = (new_read - last_read) / 2;
}

float kalman_filter(float u){
    float const r = 40, fi = 1.0, h = 1.0, q = 10.0;
    float p = 0, u_hat = 0, k = 0;
    k = p * h / (h * h * p + r);
    u_hat += k * (u - h * u_hat);
    p = (1 - k * h) * p + q;
    return u_hat;
}
void read_pixy() {
  // grab blocks!
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (j = 0; j < pixy.ccc.numBlocks; j++)
    {
      Serial.print("  block ");
      Serial.print(j);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }
}


