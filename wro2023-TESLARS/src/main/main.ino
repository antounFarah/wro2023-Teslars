#include "impu.h"
#include <SimpleKalmanFilter.h>
#include "Pixy2.h"
#include <Servo.h>

#define first_encoder_right 19
#define first_encoder_left 18
#define second_encoder_right 26
#define second_encoder_left 28
#define right_servo 10
#define left_servo 12
#define forward_servo 11
#define steering_servo 9
#define camera_servo 8
#define bts_right_enable 22
#define bts_left_enable 24
#define bts_right_pwm 6
#define bts_left_pwm 7
#define button_pin 36
#define no_of_holes 20

int ultrasonic_trigger[] = {40, 42, 44, 46};
int ultrasonic_echo[] = {2, 2, 3, 48};
int leds[] = {30, 32, 34, 38};
int analogs[] = {A8, A9, A10, A11};
bool direction; // 0 means clockwise, 1 means anti-clockwise
int start_speed = 50, low_speed = 100, high_speed = 200;
int number_of_turns = 0;

volatile int right_counter = 0; 
volatile int left_counter = 0; 

Pixy2 pixy;

int mx = 51, my = 30;
impu imu(mx, my);

SimpleKalmanFilter simpleKalmanFilter(0.5, 0.5, 0.001);

Servo ultra_servo[3];
Servo pixy_servo;
Servo steer_servo;

int ultra_servo_val[3] = {90, 90 ,90};
int pixy_servo_val = 90;
int steer_servo_val = 84;

int i, j;
float yaw, las_yaw = 0;
double delta_yaw, real_value, imu_overall_val, imu_current_val;

//volatile int measured[] = { 0, 0, 0, 0 };
volatile long duration[4];
volatile long timer[4];
volatile double distance[4];

void setup() {
    
    imu.init();
    pixy.init();
    Serial.begin(9600);
    pinMode(first_encoder_right, INPUT);
    pinMode(first_encoder_left, INPUT);
    pinMode(second_encoder_right, INPUT);
    pinMode(second_encoder_left, INPUT);

    for (i = 0; i < 4; i++) {
        pinMode(ultrasonic_trigger[i], OUTPUT);
        pinMode(ultrasonic_echo[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[0]), read_echo_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[2]), read_echo_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(first_encoder_right), count_1, RISING);
    attachInterrupt(digitalPinToInterrupt(first_encoder_left), count_2, RISING);

    for (i = 0; i < 3 ; i++)
        ultra_servo[i].attach(10 + i);
    pixy_servo.attach(camera_servo);
    steer_servo.attach(steering_servo);

    pinMode(bts_right_enable,OUTPUT);
    pinMode(bts_left_enable, OUTPUT);
    digitalWrite(bts_right_enable, HIGH);
    digitalWrite(bts_left_enable, HIGH);
    analogWrite(bts_right_pwm, 0);
    analogWrite(bts_left_pwm, 0);
    delay(10000);
}
void loop() {
    read_yaw();
    send_trig(2);
    send_trig(1);
    for (i = 0 ; i < 3; i++) ultra_servo_val[i] = imu_current_val + 90;
    refresh_servos();
    Serial.print(distance[1]);
    Serial.print("      ");
    Serial.println(distance[2]);
    delay(30);
}

void send_trig(int x) {
    // measured[x] = 1;
    digitalWrite(ultrasonic_trigger[x], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trigger[x], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trigger[x], LOW);
    delayMicroseconds(2);
    while (digitalRead(ultrasonic_echo[x]) == LOW){ continue;}
    timer[x] = micros();
}

void read_echo_1() {

    duration[1] = micros() - timer[1];
    distance[1] = (duration[1] * 0.034 / 2);
    // measured[0] = 0;
}

void read_echo_2() {
    duration[2] = micros() - timer[2];
    distance[2] = (duration[2] * 0.034 / 2);
    // measured[1] = 0;
}

double pid(double ultrasonic_value, double goal, double kp, double ki = 0, double kd = 0) {
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

void count_1() {
    right_counter += 1;
}

void count_2() {
    left_counter += 1;
}

double medianfilter(double median_values[4]) {
    //sort array by values
    for (i = 0 ; i < 4; i++){
        for (j = 4; j > i; j--){
            if (median_values[j] > median_values[i]) {
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

void runningavgfilter(double last_read, double new_read) {
    new_read = (new_read - last_read) / 2;
}

void read_yaw() {
    imu.getyaw(yaw);
    delta_yaw = yaw - las_yaw ;
    real_value += delta_yaw;
    imu_overall_val = simpleKalmanFilter.updateEstimate(real_value);
    las_yaw = yaw;
    if (direction) 
    imu_current_val = imu_overall_val - 90 * number_of_turns;
    //Serial.println(estimated_value);
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

void steer(int x) {
    if (x >= 0) {
        pixy_servo_val = map(x, 0, 100, 84, 160);
    } else {
        pixy_servo_val = map(x, 0, -100, 84, 55);
    }
}

void turn_90() {

    if(direction) {
        number_of_turns--;
    } else {
        number_of_turns++;
    }
    
}

void refresh_servos() {
    ultra_servo[0].write(ultra_servo_val[0]);
    ultra_servo[1].write(ultra_servo_val[1]);
    ultra_servo[2].write(ultra_servo_val[2]);
    steer_servo.write(steer_servo_val);
    pixy_servo.write(pixy_servo_val);
}

void moveCar(bool directionOfCar, int speedOfcar) {
    if(directionOfCar) {
        analogWrite(bts_left_pwm, 0);
        analogWrite(bts_right_pwm, speedOfcar);
    } else {
        analogWrite(bts_right_pwm, 0);
        analogWrite(bts_left_pwm, speedOfcar);
    }
}