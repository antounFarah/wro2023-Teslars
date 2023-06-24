#include "impu.h"
#include <SimpleKalmanFilter.h>
#include "Pixy2.h"
#include <Servo.h>

#define F_encoder_Right 19
#define F_encoder_Left 18
#define S_encoder_Right 38
#define S_encoder_Left 36
#define right_servo 10
#define left_servo 12
#define forward_servo 11
#define steering_servo 9
#define camera_servo 8
#define BTS_right_enabl 22
#define BTS_left_enabl 24
#define BTS_right_pwm 6
#define BTS_left_pwm 7
#define BUTTON_PIN 48

int ultrasonic_trigger[] = {22, 24, 26, 28};
int ultrasonic_echo[] = {2, 2, 3, 30};
int LED[] = {40, 42, 44, 46};
int ANALOG[] = {A8, A9, A10, A11};


#define no_of_holes 20
volatile int right_counter = 0; 
volatile int left_counter = 0; 

Pixy2 pixy;

int mx = 51, my = 30;
impu imu(mx, my);

SimpleKalmanFilter simpleKalmanFilter(0.5, 0.5, 0.001);

Servo servo1;
Servo servo2;
Servo servo3;
int i, j;

double yaw, last_yaw = 0, deltayaw, real_value, estimated_value;

volatile int measured[] = { 0, 0, 0, 0 };
volatile long duration[4];
volatile long timer[4];
volatile double distance[4];

void setup()
{
    imu.init();
    pixy.init();

    pinMode(F_encoder_Right, INPUT);
    pinMode(F_encoder_Left, INPUT);
    pinMode(S_encoder_Right, INPUT);
    pinMode(S_encoder_Left, INPUT);

    for (i = 0; i < 4; i++) {
        pinMode(ultrasonic_trigger[i], OUTPUT);
        pinMode(ultrasonic_echo[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[0]), read_echo_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[2]), read_echo_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[3]), read_echo_3, FALLING);

    attachInterrupt(digitalPinToInterrupt(F_encoder_Right), count_1, RISING);
    attachInterrupt(digitalPinToInterrupt(F_encoder_Left), count_2, RISING);

    servo1.attach(8);
    servo2.attach(9);
    servo3.attach(10);
}
void loop()
{

}

void send_trig(int x)
{
    measured[x] = 1;
    digitalWrite(ultrasonic_trigger[x], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trigger[x], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trigger[x], LOW);
    delayMicroseconds(2);
    while (digitalRead(ultrasonic_echo[x]) == LOW) { }
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

double pid(double ultrasonic_value, double goal, double kp, double ki = 0, double kd = 0)
{
    double angle, propotional, integral = 0, derivediv, error, last_error = 0, i = 0;
    propotional = (ultrasonic_value - goal) * kp;
    error = ultrasonic_value - goal;
    i = integral + error;
    integral = i * ki;
    derivediv = (error - last_error) * kd;

    angle = propotional + integral + derivediv;
    last_error = error;
    return angle;
}
void count_1(){
    right_counter += 1;
}
void count_2(){
    left_counter += 1;
}
double medianfilter(double median_values[4]){
    //sort array by values
    for (i = 0 ; i < 4; i++){
        for (j = 4; j > i; j--){
            if (median_values[j] > median_values[i]){
                //swap values
                median_values[i] += median_values[j];
                median_values[j] = median_values[i] - median_values[j];
                median_values[i] = median_values[i] - median_values[j];
            }
        }
    }
    //return the middle value 
    return median_values[2];
}
void runningavgfilter(double last_read, double new_read){
    new_read = (new_read - last_read) / 2;
}
void move_servo(Servo servo){
    imu.getyaw(yaw);
    yaw += 180;
    deltayaw = yaw - last_yaw ;
    if ((deltayaw < 270) && (deltayaw > -270)){
        real_value += deltayaw;
        estimated_value = simpleKalmanFilter.updateEstimate(real_value);
        servo.write(int(estimated_value) + 90);
    }
    Serial.println(estimated_value);
    last_yaw = yaw;
}
void read_pixy() {
  // grab blocks!
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    for (j = 0; j < pixy.ccc.numBlocks; j++)
    {
      Serial.print("  block ");
      Serial.print(j);
      Serial.print(": ");
      pixy.ccc.blocks[j].print();
    }
  }
}


