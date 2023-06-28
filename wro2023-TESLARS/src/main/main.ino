//including needed libraries 
#include "impu.h"
#include "Pixy2.h"
#include <SimpleKalmanFilter.h>
#include <Servo.h>
#include <math.h>

//defining ports names
#define encoder_right_1     18
#define encoder_right_2     28
#define encoder_left_1      19
#define encoder_left_2      26
#define right_servo         10
#define left_servo          12
#define forward_servo       11
#define steering_servo      9    
#define camera_servo        8
#define bts_right_enable    22
#define bts_left_enable     24
#define bts_right_pwm       6
#define bts_left_pwm        7
#define button_pin          36
#define no_of_holes         20

int ultrasonic_trigger[] = {40, 42, 44, 46};
int ultrasonic_echo[]    = { 2, 18,  3, 48};
int leds[]               = {30, 32, 34, 38};
int analogs[]            = {A8, A9, A10, A11};

//objects 
Pixy2 pixy;
int mx = 51, my = 30;
impu imu(mx, my);
SimpleKalmanFilter kalman(0.5, 0.5, 0.001);
Servo ultra_servo[3]; // 0 is right (max: 156, min: 54) 1 is left (max: 140, min: 20) 2 is forward (max: 120, min 76)
Servo pixy_servo;
Servo steer_servo;

//variables 
//positioning
bool direction = 1; // 0 means clockwise, 1 means anti-clockwise
int start_speed = 50, low_speed = 100, high_speed = 200;
int speed_of_car = 0;
int number_of_turns = 0;
double cm_per_hole = 0.5;
double acceleration_measured_distance_cm = 10;
double acceleration_holes = acceleration_measured_distance_cm / cm_per_hole;
double change_speed = high_speed - low_speed;
double change_per_hole = change_speed / acceleration_holes;

// ultrasonic  PID
double p_steer = 0.7;
double i_steer = 0.0;
double d_steer = 0.0;
double t_steer = 3;
double error = 0;
double last_error = 0;
double sum_error = 0;

// imu PID
double imu_p_steer = 0.7;
double imu_i_steer = 0.0;
double imu_d_steer = 0.0;
double imu_t_steer = 20;
double imu_error = 0;
double imu_last_error = 0;
double imu_sum_error = 0;

int turn_delay = 10;
double steer_adjust = 70;

//counting mean that wheel has turned
volatile int right_counter = 0; 
// volatile int left_counter = 0;

//servos are turn based on this values 
int ultra_servo_mid_val[3] = {96, 100 ,97}; // right, left, forward
int ultra_servo_val[3] = {96, 100 ,97};
int pixy_servo_val = 90;
int steer_servo_val = 84;

int i, j; // for loops variables

// MPU6050 module values
float yaw, las_yaw = 0;
double delta_yaw, real_value, imu_overall_val, imu_current_val;

//arrays are used to calculate the measured_distance between the ultrasonic and the wall
volatile bool is_measured[] = { true, true, true, true };
volatile long duration[4];
volatile long timer[4];
volatile double measured_distance[4];
double measured_distance_by_time_FB[2][7] = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};
double measured_distance_by_time_RL[2][3] = {{0, 0, 0}, {0, 0, 0}};

void setup() {
    imu.init();
    pixy.init();
    Serial.begin(9600);
    pinMode(encoder_right_1, INPUT);
    pinMode(encoder_left_1, INPUT);
    pinMode(encoder_right_2, INPUT);
    pinMode(encoder_left_2, INPUT);
    pinMode(button_pin, INPUT_PULLUP);

    for (i = 0; i < 4; i++) {
        pinMode(ultrasonic_trigger[i], OUTPUT);
        pinMode(ultrasonic_echo[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[0]), read_echo_0, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[1]), read_echo_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[2]), read_echo_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder_right_1), count_1, CHANGE);

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
}
void loop() {
    while (!digitalRead(button_pin));
    while (digitalRead(button_pin));
    //test servos
    while (!digitalRead(button_pin));
    while (digitalRead(button_pin));
    refresh_servos();
    // follow wall by ultrasonic right 
    while (!digitalRead(button_pin)); 
    while (digitalRead(button_pin));
    follow_wall(80, 10, 0);
    // follow wall by ultrasonic left 
    while (!digitalRead(button_pin)); 
    while (digitalRead(button_pin));
    follow_wall(80, 10, 1);
    // turning 90 degrees
    direction = false;
    while (!digitalRead(button_pin));
    while (digitalRead(button_pin));
    turn_90();
    // turning 90 degrees
    direction = true;
    while (!digitalRead(button_pin));
    while (digitalRead(button_pin));
    turn_90();
}

// // // //
// // // // // 
// ultrasonic_block
// // // // //
// // // //

void send_trig(int ultrasonic_index) {
    if (!is_measured[ultrasonic_index] || micros() - timer[ultrasonic_index] < 30000) return;
    if (ultrasonic_index >= 2){
        int length_of_array = sizeof(measured_distance_by_time_FB[ultrasonic_index]) / sizeof(measured_distance_by_time_FB[ultrasonic_index][0]);
        for (i = length_of_array; i > 0; i--){
            measured_distance_by_time_FB[ultrasonic_index][i] = measured_distance_by_time_FB[ultrasonic_index][i - 1];
        }
    } else {
        int length_of_array = sizeof(measured_distance_by_time_RL[ultrasonic_index]) / sizeof(measured_distance_by_time_RL[ultrasonic_index][0]);
        for (i = length_of_array; i > 0; i--){
            measured_distance_by_time_RL[ultrasonic_index][i] = measured_distance_by_time_RL[ultrasonic_index][i - 1];
        }
    }
    is_measured[ultrasonic_index] = false;
    digitalWrite(ultrasonic_trigger[ultrasonic_index], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trigger[ultrasonic_index], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trigger[ultrasonic_index], LOW);
    delayMicroseconds(2);
    while (digitalRead(ultrasonic_echo[ultrasonic_index]) == LOW) { continue;}
    timer[ultrasonic_index] = micros();
}

void read_echo_0() {

    duration[0] = micros() - timer[0];
    measured_distance[0] = (duration[0] * 0.034 / 2);
    is_measured[0] = true;
    measured_distance_by_time_RL[0][0] = measured_distance[0];
}
void read_echo_1() {
    duration[1] = micros() - timer[1];
    measured_distance[1] = (duration[1] * 0.034 / 2);
    is_measured[1] = true;
    measured_distance_by_time_RL[1][0] = measured_distance[0];
}
void read_echo_2() {
    duration[2] = micros() - timer[2];
    measured_distance[2] = (duration[2] * 0.034 / 2);
    is_measured[2] = true;
    measured_distance_by_time_FB[0][0] = measured_distance[0];
}

double find_distance(int ultrasonic_index) {
    if (ultrasonic_index >= 2){
        int length_of_array = sizeof(measured_distance_by_time_FB[ultrasonic_index]) / sizeof(measured_distance_by_time_FB[ultrasonic_index][0]);
        return find_median(measured_distance_by_time_FB[ultrasonic_index], length_of_array);
    } else {
        int length_of_array = sizeof(measured_distance_by_time_RL[ultrasonic_index]) / sizeof(measured_distance_by_time_RL[ultrasonic_index][0]);
        return find_median(measured_distance_by_time_RL[ultrasonic_index], length_of_array);
    }
}

double find_median(double some_array[], int length_of_array) {
    double temp_array[length_of_array];
    for (i = 0; i < length_of_array; i++) {
        temp_array[i] = some_array[i];
    }
    for (i = 0 ; i < length_of_array ; i++){
        for (j = length_of_array ; j > i; j--){
            if (temp_array[j] > temp_array[i]) {
                //swap values
                temp_array[i] += temp_array[j];
                temp_array[j] = temp_array[i] - temp_array[j];
                temp_array[i] = temp_array[i] - temp_array[j];
            }
        }
    }
    return temp_array[length_of_array / 2];
}

void clean_all_measured_distance_arrays() {
    clean_array(measured_distance_by_time_FB[0]);
    clean_array(measured_distance_by_time_FB[1]);
    clean_array(measured_distance_by_time_RL[0]);
    clean_array(measured_distance_by_time_RL[1]);
}

void clean_array(double some_array[]) {
    int length_of_array = sizeof(some_array) / sizeof(some_array[0]);
    for (i = 0; i < length_of_array; i++) {
        some_array[i] = 0;
    }
}

// // // //
// // // // // 
// end_of_ultrasonic_block
// // // // //
// // // //

void count_1() {
    right_counter += 1;
}

void read_yaw() {
    imu.getyaw(yaw);
    delta_yaw = yaw - las_yaw ;
    real_value += delta_yaw;
    imu_overall_val = real_value;//kalman.updateEstimate(real_value);
    las_yaw = yaw;
    if (direction) {
        imu_current_val = imu_overall_val + 90 * number_of_turns;
    } else {
        imu_current_val = imu_overall_val - 90 * number_of_turns;
    }
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


void turn_90() {
    moveCar(1, low_speed);
    steer(0);
    refresh_servos();
    double temp_imu_current_val = imu_current_val;
    if(direction) {
        while ((imu_current_val - temp_imu_current_val) > -15) {
            refresh_servos();
            steer(-10 + (imu_current_val - temp_imu_current_val) * 90 / 15);
        }
        steer(-100);
        while (imu_current_val > -75) {
            refresh_servos();
        }
         while (imu_current_val > -90) {
            refresh_servos();
            steer((imu_current_val + 90) * (-100 / 15));
        }
        number_of_turns++;
    } else {
        while ((imu_current_val - temp_imu_current_val) < 15) {
            refresh_servos();
            steer(10 + (imu_current_val - temp_imu_current_val) * 90 / 15);
        }
        steer(100);
        while (imu_current_val < 75) {
            refresh_servos();
        }
        while (imu_current_val < 90) {
            refresh_servos();
            steer((imu_current_val - 90) * (-100 / 15));
        }
        number_of_turns--;
    }
    steer(0);
    clean_all_measured_distance_arrays();
}

void refresh_servos() {
    read_yaw();
    for (i = 0; i < 3; i++) {
        ultra_servo_val[i] = ultra_servo_mid_val[i] + imu_current_val; 
    }
    if (ultra_servo_val[0] > 156) ultra_servo_val[0] = 156;
    if (ultra_servo_val[0] <  54) ultra_servo_val[0] = 54;
    if (ultra_servo_val[1] > 140) ultra_servo_val[1] = 140;
    if (ultra_servo_val[1] <  20) ultra_servo_val[1] = 20;
    if (ultra_servo_val[2] > 120) ultra_servo_val[2] = 120;
    if (ultra_servo_val[2] <  76) ultra_servo_val[2] = 76;
    ultra_servo[0].write(ultra_servo_val[0]);
    ultra_servo[1].write(ultra_servo_val[1]);
    ultra_servo[2].write(ultra_servo_val[2]);
    steer_servo.write(steer_servo_val);
    pixy_servo.write(pixy_servo_val);
}

void steer(double x) {
    if (x >= 0) {
        steer_servo_val = map(x, 0, 100, 84, 160);
    } else {
        steer_servo_val = map(x, 0, -100, 84, 55);
    }
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

void follow_wall(double finishmeasured_distance,double measured_distanceFromWall, bool wall) {
    speed_of_car = low_speed;
    steer(0);
    moveCar(true, speed_of_car);
    send_trig(wall);
    send_trig(2);
    refresh_servos();
    while(!find_distance(wall) || !find_distance(2)) {
        send_trig(wall);
        send_trig(2);
    }
    right_counter = 0;
    while (find_distance(2) > finishmeasured_distance) {
        Serial.println(find_distance(2));
        refresh_servos();
        send_trig(2);
        send_trig(wall);
        accelerate_car(find_distance(2) - finishmeasured_distance);
        pid_steer_ultra(wall, measured_distanceFromWall);
    }
}

void accelerate_car(int untilfinishmeasured_distance) {
    double moved_measured_distance = right_counter / no_of_holes * 6 * PI;
    if (speed_of_car < high_speed && untilfinishmeasured_distance > acceleration_measured_distance_cm) {
        speed_of_car = low_speed + right_counter * change_per_hole;
    } else if (untilfinishmeasured_distance < acceleration_measured_distance_cm) {
        if (speed_of_car <= high_speed) {
            double remaining_holes = untilfinishmeasured_distance / cm_per_hole;
            if (remaining_holes / right_counter > 1) {
                speed_of_car = low_speed + right_counter * change_per_hole;
            } else {
                speed_of_car = low_speed + remaining_holes * change_per_hole;
            }
        }
    }
}

void pid_steer_ultra(bool wall, double measured_distanceFromWall) {
    int a = -1;
    if(wall) a = 1;
    error = a * (find_distance(wall) - measured_distanceFromWall);
    error *= 100;
    error /= t_steer;
    sum_error += error;
    double pid_val = p_steer * error + d_steer * (error - last_error) + i_steer * sum_error;
    last_error = error;
    steer(pid_val);
}

void pid_steer_imu(double angle) {
    refresh_servos();
    imu_error = imu_current_val - angle;
    imu_error *= imu_t_steer;
    imu_sum_error += imu_error;
    double imu_pid_val = imu_p_steer * imu_error + imu_d_steer * (imu_error - imu_last_error) + imu_i_steer * imu_sum_error;
    imu_last_error = imu_error;
    steer(imu_pid_val);
}




/*void turn_90() {
    moveCar(1, low_speed);
    steer(0);
    refresh_servos();
    if(direction) {
        for (i = 1; i <= 10; i++) {
            steer(-1 * 10 * i);
            refresh_servos();
            if (i < 10) delay(turn_delay);
        }
        while (imu_current_val > -1 * steer_adjust) {
            refresh_servos();
        }
        for (i = 9; i >= 0; i--) {
            steer(-1 * 10 * i);
            refresh_servos();
            if (i > 0) delay(turn_delay);
        }
        number_of_turns++;
    } else {
        for (i = 1; i <= 10; i++) {
            steer(10 * i);
            refresh_servos();
            if (i < 10) delay(turn_delay);;
        }
        while (imu_current_val < steer_adjust) {
            refresh_servos();
        }
        for (i = 9; i >= 0; i--) {
            steer(10 * i);
            refresh_servos();
            if (i > 0) delay(turn_delay);
        }
        number_of_turns--;
    }*/