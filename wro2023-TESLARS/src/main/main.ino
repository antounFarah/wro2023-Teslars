//including needed libraries 
#include "impu.h"                   //including our own library which include MPU6050.h library with DMP (Digital Motion Processor)
#include "Pixy2.h"                  //including pixy v2.1 camera library 
#include <SimpleKalmanFilter.h>     //including kalman filter library to make MPU6050 values more reliable
#include <Servo.h>                  //including servo library to control servos
#include <math.h>                   //including math library to use functions like abs(), map(), etc..

//defining ports names
// #define encoder_right_1     00   //right encoder phase 1 should be connected to interrupt pin 
// #define encoder_right_2     00   //right encoder phase 2 should be connected to interrupt||input pin
// #define encoder_left_1      00   //left encoder phase 1 should be connected to interrupt pin
// #define encoder_left_2      00   //left encoder phase 2 should be connected to interrupt||input pin
#define right_servo         10      //sg-90 servo that holds the right ultrasonic pin
#define left_servo          12      //sg-90 servo that holds the left ultrasonic pin
#define forward_servo       11      //sg-90 servo that holds the forward ultrasonic pin
#define steering_servo      9       //mg-995 servo that controls the ackerman steering
#define camera_servo        5       //mg-995 servo that holds the pixyCam
#define bts_right_enable    22      //connected to bts-7960B r-en pin
#define bts_left_enable     24      //connected to bts-7960B l-en pin
#define bts_right_pwm       6       //connected to bts-7960B r-pwm pin
#define bts_left_pwm        7       //connected to bts-7960B l-pwm pin
#define button_pin          36      //the button to start the car
#define no_of_holes         20      //a variable to specify the holes in the disk of the encoder

int ultrasonic_trigger[] = {40, 42, 44, 46};    //the four ultrasonic trigger pins {right, left, forward, backward}
int ultrasonic_echo[]    = { 18, 19,  2, 3};    //the four ultrasonic echo pins {right, left, forward, backward}
int leds[]               = {30, 32, 34, 38};    //four leds to indicate it is time to recharge batteries
int analogs[]            = {A8, A9, A10, A11};  //four power sourced connected to voltage dividers

//objects 
Pixy2 pixy;                                 //declaring pixyCam object
int mx = 51, my = 30;                       //imu mpu6050 offsets used for calibration
impu imu(&mx, &my);                         //imu mpu6050 offsets used for calibration
SimpleKalmanFilter kalman(0.5, 0.5, 0.001); //declaring kalman_filter object
Servo ultra_servo[3];   // 0 is right (max: 156, min: 54) 1 is left (max: 140, min: 20) 2 is forward (max: 120, min 76)
Servo pixy_servo; 
Servo steer_servo;

//variables 

//positioning
bool direction = 1; // 0 means clockwise, 1 means anti-clockwise
int start_speed = 140, low_speed = 200, high_speed = 230;
int speed_of_car = 0;   
int number_of_turns = 0;    //number of 90* turns
double cm_per_hole = 0.5;   //each hole on the encoder is equal to this value
double acceleration_measured_distance_cm = 10;  //distance to accelerate
double acceleration_holes = acceleration_measured_distance_cm / cm_per_hole;
double change_speed = high_speed - low_speed;
double change_per_hole = change_speed / acceleration_holes;

//ultrasonic PID values
double p_steer = 5.5;
double i_steer = 0.0;
double d_steer = 70;
double error = 0;
double last_error = 0;
double sum_error = 0;
double pid_val = 0;

//imu PID values
double imu_p_steer = 3;
double imu_i_steer = 0.0;
double imu_d_steer = 0.0;
double imu_error = 0;
double imu_last_error = 0;
double imu_sum_error = 0;

//counting mean that wheel has turned one hole
volatile int left_counter = 0;
volatile int right_counter = 0;
// volatile double avg_counter = 0;

//servos are turn based on this values 
int ultra_servo_mid_val[3] = {97, 85 ,98};  //{right, left, forward} middle values
int ultra_servo_val[3] = {97, 85 ,98};      //global values for the position of the servos, starting at middle value
int pixy_servo_val = 90;            //middle value for pixy servo
int pixy_servo_val_right = 125;     //maximum anti-clockwise value for pixy servo
int pixy_servo_val_left = 60;       //maximum clockwise value for pixy servo
int steer_servo_val_middle = 83;    //middle value for steer servo
int steer_servo_val_right = 125;    //maximum anti-clockwise value for steer servo
int steer_servo_val_left = 60;      //maximum clockwise value for steer servo
int steer_servo_val = steer_servo_val_middle;

// MPU6050 module values
float yaw, last_yaw = 0;
double delta_yaw, real_value = 0;
double imu_overall_val = 0;         //yaw overall value from the starting position range from -360 * 3 to 360 * 3 
double imu_current_val = 0;         //yaw straight section value range from -90 to 90

//arrays used to calculate the measured_distance between the ultrasonic and the wall
volatile bool is_measured[] = { true, true, true, true };
volatile long duration[4];
volatile long timer[4];
volatile double measured_distance[4];
double measured_distance_by_time_FB[2][7] = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}}; //median values for forward and backward hc-sr04
double measured_distance_by_time_RL[2][5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}}; //median values for right and left hc-sr04
//the next tow arrays are used to ensure that the median filter function doesn't work unless
//there is a new value from the hc-sr04 to prevent duplicates
bool interrupt_state[4] = {0, 0, 0, 0};         //every tinme echo interrupt it changes this value
bool last_interrupt_state[4] = {0, 0, 0, 0};    //to remember the interrupt_state

//avoiding variables
long object_center_x, object_center_y, object_width, object_height;
int object_color, object_color_array[5];
double middle_distance = 42.5;
int color_of_object = 0;
bool start_position = 0; // 0 means last one was red, 1 means last one was green.
int first_obstacle_distance = 65;
double avoid_start_distance_g = 0;
double avoid_diagonal_distance_1_g = 40;
double avoid_diagonal_distance_2_g = 0;
double avoid_middle_distance_g = 55;
double avoid_finish_distance_g = 0;
double avoid_start_distance_r = 0;
double avoid_diagonal_distance_1_r = 45;
double avoid_diagonal_distance_2_r = 0;
double avoid_middle_distance_r = 55;
double avoid_finish_distance_r = 0;
double middle_finish_distance = 40;
double angle_of_avoid = 60;
int avoid_object_ultrasonic_delay = 500;
double distance_from_object = 25;

void setup() {
    imu.init();         //intializing mpu6050
    pixy.init();        //intializing pixyCam
    Serial.begin(115200);
    // pinMode(encoder_right_1, INPUT);
    // pinMode(encoder_left_1, INPUT);
    // pinMode(encoder_right_2, INPUT);
    // pinMode(encoder_left_2, INPUT);
    pinMode(button_pin, INPUT_PULLUP);
    //hc-sr04 pins and interrupts
    for (int i = 0; i < 4; i++) {
        pinMode(ultrasonic_trigger[i], OUTPUT);
        pinMode(ultrasonic_echo[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[0]), ultra_R_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[1]), ultra_L_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[2]), ultra_F_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[3]), ultra_B_interrupt, FALLING);
    // attachInterrupt(digitalPinToInterrupt(encoder_right_1), counter_right_interrupt, FALLING);
    // attachInterrupt(digitalPinToInterrupt(encoder_left_1), counter_left_interrupt, FALLING);
    //attach servos
    for (int i = 0; i < 3 ; i++)
        ultra_servo[i].attach(10 + i);
    pixy_servo.attach(camera_servo);
    steer_servo.attach(steering_servo);
    //bts-7960B pins
    pinMode(bts_right_enable,OUTPUT);
    pinMode(bts_left_enable, OUTPUT);
    digitalWrite(bts_right_enable, HIGH);
    digitalWrite(bts_left_enable, HIGH);
    analogWrite(bts_right_pwm, 0);
    analogWrite(bts_left_pwm, 0);
}

void loop() {
    // pixy.ccc.getBlocks();
    // if (pixy.ccc.numBlocks > 0) pixy.ccc.blocks[0].print();
    while (!digitalRead(button_pin));
    while (digitalRead(button_pin));

    // move_car(1, low_speed);
    // pid_steer_imu_encoder(35, 0);
    // pid_steer_imu_encoder(avoid_diagonal_distance_1_g, -60);
    // pid_steer_imu_encoder(avoid_middle_distance_g, 0);
    // pass_the_corner(2);
    
    // while (1){
    //     test_car(1);
    // }

    // follow_wall(50,40,!direction);
    int position_after_passing  = pass_the_section();
    pass_the_corner(position_after_passing);
    // turn_90_reverse();
    move_car(0,0);
    clean_all_measured_distance_arrays();
    
}

//function to test functionality of the car
void test_car(int a) {
    if (a == 1) {
        refresh_servos();
        Serial.print(yaw);
        delay(10);
        steer(-100);
        // move_car(1,255);
        send_trig(0);
        send_trig(1);
        send_trig(2);
        send_trig(3);
        Serial.print("\t\t\t\t");
        Serial.print(find_distance(0));
        Serial.print("\t\t\t\t");
        Serial.print(find_distance(1));
        Serial.print("\t\t\t\t");
        Serial.print(find_distance(2));
        Serial.print("\t\t\t\t");
        Serial.print(find_distance(3));
        Serial.println("\t\t\t\t");
    }
    if (a == 2) { 
        move_car(1,225);
        Serial.println(right_counter);
        // Serial.print("\t\t\t");
        // Serial.print(left_counter);
        // Serial.print("\t\t\t");
        // Serial.println(right_counter);
    }
}

// // // //
// // // // // 
// ultrasonic_block
// // // // //
// // // //


void send_trig(int ultrasonic_index) {
    //don't send triggers if you didn't get the last echo or it has been lass than 35 ms
    if (!is_measured[ultrasonic_index] || micros() - timer[ultrasonic_index] < 35000) return;
    is_measured[ultrasonic_index] = false;  //when the echo return it will be high again so we can send trig again
    //this is how to send trigger according to hc-sr04 datasheet
    digitalWrite(ultrasonic_trigger[ultrasonic_index], LOW);
    delayMicroseconds(4);
    digitalWrite(ultrasonic_trigger[ultrasonic_index], HIGH);
    delayMicroseconds(12);
    digitalWrite(ultrasonic_trigger[ultrasonic_index], LOW);
    delayMicroseconds(4);
    //if the hc-sr04 did'nt send a trigger for some reason like a voltage drop or getting too mant interrupts
    //send trigger again after 1 ms (when the trigger is sent the echo pin changes to high)
    double anti_stall_timer = micros();
    while (digitalRead(ultrasonic_echo[ultrasonic_index]) == LOW && micros() - anti_stall_timer < 1000) { continue; }
    if (micros() - anti_stall_timer >= 1000) is_measured[ultrasonic_index] = true;  //to be able to send trigger again
    else timer[ultrasonic_index] = micros();    //get the time stamp for when the trigger got sent
}

// 
//interrupt function for calculating the distace by the time and the speed of sound in the air 
//Right sensor
void ultra_R_interrupt(){
    duration[0] = micros() - timer[0];
    measured_distance[0] = (duration[0] * 0.034 / 2);
    is_measured[0] = true;
    interrupt_state[0] = !interrupt_state[0];
}
//left sensor
void ultra_L_interrupt() {
    duration[1] = micros() - timer[1];
    measured_distance[1] = (duration[1] * 0.034 / 2);
    is_measured[1] = true;
    interrupt_state[1] = !interrupt_state[1];
}
//forward sensor
void ultra_F_interrupt(){
    duration[2] = micros() - timer[2];
    measured_distance[2] = (duration[2] * 0.034 / 2);
    is_measured[2] = true;
    interrupt_state[2] = !interrupt_state[2];
}
//backward sensor
void ultra_B_interrupt() {
    duration[3] = micros() - timer[3];
    measured_distance[3] = (duration[3] * 0.034 / 2);
    is_measured[3] = true;
    interrupt_state[3] = !interrupt_state[3];
}

double find_distance(int ultrasonic_index) {
    //moving the ultrasonic arrays one index to make space for the new reading
    //they are sorted by the time they got in
    if (ultrasonic_index >= 2) { //for the forward and backward arrays
        int length_of_array = sizeof(measured_distance_by_time_FB[ultrasonic_index - 2]) / sizeof(measured_distance_by_time_FB[ultrasonic_index - 2][0]);
        if (last_interrupt_state[ultrasonic_index] != interrupt_state[ultrasonic_index]) { //check if you got a new value first
            for (int i = length_of_array - 1; i > 0; i--){ //moving elements one index
                measured_distance_by_time_FB[ultrasonic_index - 2][i] = measured_distance_by_time_FB[ultrasonic_index - 2][i - 1];
            }
            if (measured_distance[ultrasonic_index] > 350) {  //if you get a value over 350 consider it 0 (we don't have distances like that so must be wrong)
                measured_distance_by_time_FB[ultrasonic_index - 2][0] = 0;
            }
            else {  //get the new value in
                 measured_distance_by_time_FB[ultrasonic_index - 2][0] = measured_distance[ultrasonic_index];
            }
            last_interrupt_state[ultrasonic_index] = interrupt_state[ultrasonic_index]; //
        }
        return find_median(measured_distance_by_time_FB[ultrasonic_index - 2], length_of_array); //find the median
    } else {    //for the right and left arrays
        int length_of_array = sizeof(measured_distance_by_time_RL[ultrasonic_index]) / sizeof(measured_distance_by_time_RL[ultrasonic_index][0]);
        if (last_interrupt_state[ultrasonic_index] != interrupt_state[ultrasonic_index]) {
            for (int i = length_of_array - 1; i > 0; i--){
                measured_distance_by_time_RL[ultrasonic_index][i] = measured_distance_by_time_RL[ultrasonic_index][i - 1];
            }
            if (measured_distance[ultrasonic_index] > 350) {
                measured_distance_by_time_RL[ultrasonic_index][0] = 0;
            }
            else {
                 measured_distance_by_time_RL[ultrasonic_index][0] = measured_distance[ultrasonic_index];
            }
            last_interrupt_state[ultrasonic_index] = interrupt_state[ultrasonic_index];
        }
        return find_median(measured_distance_by_time_RL[ultrasonic_index], length_of_array);
    }
}

double find_median(double some_array[], int length_of_array) {
    //sort the arraies by values to get the median value
    double temp_array[length_of_array];
    for (int i = 0; i < length_of_array; i++) {     //duplicate the array
        temp_array[i] = some_array[i];
    }
    //sort them
    bool is_swapped;
    for (int i = 0 ; i < length_of_array - 1; i++){
        is_swapped = false;
        for (int j = 0 ; j < length_of_array - i - 1; j++){
            if (temp_array[j] > temp_array[j + 1]) {
                //swap values
                temp_array[j + 1] += temp_array[j];
                temp_array[j] = temp_array[j + 1] - temp_array[j];
                temp_array[j + 1] = temp_array[j + 1] - temp_array[j];
                is_swapped = true;
            }
        }
        if (!is_swapped) break;
    }
    //the following for testing only :)
    // for (int i = 0 ; i < length_of_array; i++){
    //     Serial.print(temp_array[i]);
    //     Serial.print("\t");
    // }
    // Serial.print("\t median:");
    return temp_array[length_of_array / 2]; //return the median value
}

void clean_all_measured_distance_arrays() {
    //setting all arrays valves to 0 
    clean_array(measured_distance_by_time_FB[0]);
    clean_array(measured_distance_by_time_FB[1]);
    clean_array(measured_distance_by_time_RL[0]);
    clean_array(measured_distance_by_time_RL[1]);
    last_error = 0;
    error = 0;
    sum_error = 0;
    imu_last_error = 0;
    imu_error = 0;
    imu_sum_error = 0;
}

void clean_array(double some_array[]) { //setting all arrays valves to 0 
    int length_of_array = sizeof(some_array) / sizeof(some_array[0]);
    for (int i = 0; i < length_of_array; i++) {
        some_array[i] = 0;
    }
}

// // // //
// // // // // 
// end_of_ultrasonic_block
// // // // //
// // // //

// void counter_right_interrupt() {
//     right_counter += 1;
//     // avg_counter += 0.5;
// }

// void counter_left_interrupt() {
//     left_counter += 1;
//     avg_counter += 0.5;
// }

void pause_interrupts() {   //pause all interrupts but the sda scl ones (to solve problems in the arduino wire library,
                            //sometimes it get stuck in an infinte loop if you got too many interrupts)
    detachInterrupt(ultrasonic_echo[0]);
    detachInterrupt(ultrasonic_echo[1]);
    detachInterrupt(ultrasonic_echo[2]);
    detachInterrupt(ultrasonic_echo[3]);
    // detachInterrupt(encoder_right_1);
}

void resume_interrupts() { //resume interrupts
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[0]), ultra_R_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[1]), ultra_L_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[2]), ultra_F_interrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ultrasonic_echo[3]), ultra_B_interrupt, FALLING);
    //attachInterrupt(digitalPinToInterrupt(encoder_right_1), counter_right_interrupt, FALLING);
}

void read_yaw() {
    //pause_interrupts();
    imu.getyaw(yaw);        //get the value of yaw
    //resume_interrupts();
    delay(5);               //so it won't ask the dmp for too many values in a short time
    delta_yaw = yaw - last_yaw ;    //get the delta of yaw value
    if (delta_yaw < 150 && delta_yaw > -150) { //solving the problem that the dmp return values 180 and -180 in a random
        real_value += delta_yaw;
        imu_overall_val = real_value; // kalman.updateEstimate(real_value);
    }
    last_yaw = yaw;
    imu_current_val = imu_overall_val + 90 * number_of_turns;   // get the yaw straight section value range to be  from -90 to 90
    // Serial.print(imu_overall_val);
    // Serial.print("\t");
    // Serial.println(imu_current_val);
}

void turn_90_reverse() { //turn to the next section in reverse
    int temp_direction;
    if (direction) temp_direction = -1;
    else temp_direction = 1;
    move_car(1, start_speed);
    while (abs(imu_current_val) < 30) {
        steer(temp_direction * 100);
        refresh_servos();
    }
    move_car(0,0);
    steer(-1 * temp_direction * 100);
    delay(200);
    move_car(0,start_speed);
    while (abs(imu_current_val) < 85) {
        steer(-1 * temp_direction * 100);
        refresh_servos();
    }
    number_of_turns -= temp_direction;
    refresh_servos();
    steer(0);
    move_car(0, 0);
    refresh_servos();
}

void turn_90_short(){   //turn to the next section fast way
    int temp_direction;
    if (direction) temp_direction = -1;
    else temp_direction = 1;
    move_car(1, low_speed);
    while (abs(imu_current_val) < 20) {
        steer(temp_direction * 80);
        refresh_servos();
    }
    while (abs(imu_current_val) < 70) {
        steer(temp_direction * 100);
        refresh_servos();
    }
    while (abs(imu_current_val) < 85) {
        steer(temp_direction * 80);
        refresh_servos();
    }
    number_of_turns -= temp_direction;
}

void turn_90() {    //turn to the next section slow way
    move_car(1, low_speed - 15);
    steer(0);
    refresh_servos();
    double temp_imu_current_val = imu_current_val;
    if(direction) {
        while ((imu_current_val - temp_imu_current_val) > -15) {
            refresh_servos();
            steer(-60 + (imu_current_val - temp_imu_current_val) * 40 / 15);
        }
        steer(-100);
        number_of_turns++;
        refresh_servos();
        while (imu_current_val > 30) { // > 20
            refresh_servos();
        }
        while (imu_current_val > 10) { // > 0
            refresh_servos();
            steer((imu_current_val - 10) * -70 / 20 -30);
        }
    } else {
        while ((imu_current_val - temp_imu_current_val) < 15) {
            refresh_servos();
            steer(60 + (imu_current_val - temp_imu_current_val) * 40 / 15);
        }
        steer(100);
        number_of_turns--;
        refresh_servos();
        while (imu_current_val  < - 30) { // < -20
            refresh_servos();
        }
        while (imu_current_val  < -10) { // < 0
            refresh_servos();
            steer((imu_current_val + 10) * -70 / 20 +30);
        }
    }
    steer(0);
    refresh_servos();
    clean_all_measured_distance_arrays();
}

void refresh_servos() {     //refresh servos according to the imu so they will be perpendicular to the walls and move the steering servo to the new value
    read_yaw();
    for (int i = 0; i < 3; i++) {
        ultra_servo_val[i] = ultra_servo_mid_val[i] + imu_current_val; 
    }
    if (ultra_servo_val[0] > 156) ultra_servo_val[0] = 156;
    if (ultra_servo_val[0] <  65) ultra_servo_val[0] = 65;
    if (ultra_servo_val[1] > 120) ultra_servo_val[1] = 120;
    if (ultra_servo_val[1] <  20) ultra_servo_val[1] = 20;
    if (ultra_servo_val[2] > 125) ultra_servo_val[2] = 125;
    if (ultra_servo_val[2] <  75) ultra_servo_val[2] = 75;
    ultra_servo[0].write(ultra_servo_val[0]);
    ultra_servo[1].write(ultra_servo_val[1]);
    ultra_servo[2].write(ultra_servo_val[2]);
    pixy_servo_val = 90 + imu_current_val;
    if (pixy_servo_val > 140) pixy_servo_val = 140;
    if (pixy_servo_val <  40) pixy_servo_val = 40;
    steer_servo.write(steer_servo_val);
    pixy_servo.write(pixy_servo_val);
    //for testing only :)
    // Serial.print("\t");
    // Serial.print(ultra_servo_val[0]);
    // Serial.print("\t");
    // Serial.print(ultra_servo_val[1]);
    // Serial.print("\t");
    // Serial.print(ultra_servo_val[2]);
    // Serial.print("\t");
    // Serial.println(imu_current_val);
}

void steer(double x) { //map the steer value so it will be from -100 to 100
    if (x >= 0) {
        if (x > 100) x = 100;
        steer_servo_val = map(x, 0, 100, steer_servo_val_middle, steer_servo_val_right);
    } else {
        if (x < -100) x = -100;
        steer_servo_val = map(x, 0, -100, steer_servo_val_middle, steer_servo_val_left);
    }
}

void move_car(bool directionOfCar, int speedOfcar) { //send signals to the bts-7960B to move the car
    if(directionOfCar) {
        analogWrite(bts_left_pwm, 0);
        analogWrite(bts_right_pwm, speedOfcar);
    } else {
        analogWrite(bts_right_pwm, 0);
        analogWrite(bts_left_pwm, speedOfcar);
    }
}

void follow_wall(double finish_distance, double distanceFromWall, bool wall) {  //follow the (right or left) wall by a distance of distanceFromWall 
    speed_of_car = low_speed;                                                   //until the distance from the forward wall reaches finish_distance
    steer(0);
    move_car(true, speed_of_car);
    send_trig(wall);
    send_trig(2);
    refresh_servos();
    double temp_forward_distance = find_distance(2);
    double temp_wall_distance = find_distance(wall);
    while(temp_wall_distance < 0.1 || temp_forward_distance < 0.1) { //make sure you are getting a valid values from the hc-sr04s
        pid_steer_imu(0);
        send_trig(wall);
        send_trig(2);
        temp_forward_distance = find_distance(2);
        temp_wall_distance = find_distance(wall);
        //for testing only :)
        // Serial.print(temp_forward_distance);
        // Serial.print("\t\t");
        // Serial.println(temp_wall_distance);


    }
    left_counter = 0;
    while (find_distance(2) > finish_distance) {
        refresh_servos();
        send_trig(2);
        send_trig(wall);
        accelerate_car(find_distance(2) - finish_distance);
        pid_steer_ultra(wall, distanceFromWall);
    }
    clean_all_measured_distance_arrays();
}

void follow_wall(double distanceFromWall, bool wall) {  //follow the (right or left) wall by a distance of distanceFromWall
    speed_of_car = low_speed;
    move_car(true, speed_of_car);
    send_trig(wall);
    refresh_servos();
    while(find_distance(wall) < 0.1) {
        pid_steer_imu(0);
        send_trig(wall);
    }
    refresh_servos();
    send_trig(wall);
    pid_steer_ultra(wall, distanceFromWall);
}

void accelerate_car(int untilfinish_distance) { //accelerate car
    if (speed_of_car < high_speed && untilfinish_distance > acceleration_measured_distance_cm) {
        speed_of_car = low_speed + left_counter * change_per_hole;
    } else if (untilfinish_distance < acceleration_measured_distance_cm) {
        if (speed_of_car <= high_speed) {
            double remaining_holes = untilfinish_distance / cm_per_hole;
            if (remaining_holes / left_counter > 1) {
                speed_of_car = low_speed + left_counter * change_per_hole;
            } else {
                speed_of_car = low_speed + remaining_holes * change_per_hole;
            }
        }
    }
}

void pid_steer_ultra(bool wall, double distanceFromWall) { //pid contoller to steer the car according to the distance from the wall the car is floolowing
    bool no_wall = 0;
    int a = 1;
    if(wall) a = -1;
    error = a * (find_distance(wall) - distanceFromWall);
    if (abs(error - last_error) > 0.01) {
        sum_error += error;
        pid_val = p_steer * error + d_steer * (error - last_error) + i_steer * sum_error;
        if (abs(error - last_error) > 10) {
            no_wall = 1;
        }
        //for testing only :)
        // Serial.print(error);
        // Serial.print("\t");
        // Serial.println(last_error);
        last_error = error;
    }
    if (pid_val > 100) pid_val = 100;
    else if (pid_val < -100) pid_val = -100;
    if (no_wall) {
        pid_steer_imu(0);
    } else {
        steer(pid_val);
    }
}

void pid_steer_imu(double angle) {      //pid contoller to steer the car according to the imu yaw value
    move_car(true, speed_of_car);
    refresh_servos();
    imu_error = imu_current_val - angle;
    imu_error *= -1;
    imu_sum_error += imu_error;
    double imu_pid_val = imu_p_steer * imu_error + imu_d_steer * (imu_error - imu_last_error) + imu_i_steer * imu_sum_error;
    imu_last_error = imu_error;
    if (imu_pid_val < -100) imu_pid_val = -100;
    if (imu_pid_val > 100) imu_pid_val = 100;
    steer(imu_pid_val);
    refresh_servos();
}

void pid_reverse_imu(double angle) {    //pid contoller to steer the car according to the imu yaw value in reverse 
    move_car(false, speed_of_car);      //(made in a seperate function in case the pid values are diffrent in reverse to tune them)
    refresh_servos();
    imu_error = imu_current_val - angle;
    imu_error *= -1;
    imu_sum_error += imu_error;
    double imu_pid_val = imu_p_steer * imu_error + imu_d_steer * (imu_error - imu_last_error) + imu_i_steer * imu_sum_error;
    imu_last_error = imu_error;
    if (imu_pid_val < -100) imu_pid_val = -100;
    if (imu_pid_val > 100) imu_pid_val = 100;
    steer(-1 * imu_pid_val);
    refresh_servos();
}

void pid_steer_imu_encoder(double finish_distance, double angle) {  //pid contoller to steer the car according to the imu yaw value until you drive a certin distance calculated from the encoder
    move_car(true, low_speed);
    right_counter = 0;
    while (right_counter < finish_distance) {
        refresh_servos();
        imu_error = imu_current_val - angle;
        imu_error *= -1;
        imu_sum_error += imu_error;
        double imu_pid_val = imu_p_steer * imu_error + imu_d_steer * (imu_error - imu_last_error) + imu_i_steer * imu_sum_error;
        imu_last_error = imu_error;
        if (imu_pid_val < -60) imu_pid_val = -60;
        if (imu_pid_val > 60) imu_pid_val = 60;
        steer(imu_pid_val);
        refresh_servos();
    }
}

void pid_steer_imu(double finish_distance, double angle) { //pid contoller to steer the car according to the imu yaw value until the forward distance is equal to finish_distance
    move_car(true, low_speed);
    double distance_from_right_wall = find_distance(0);
    double distance_from_left_wall = find_distance(1);
    while(find_distance(2) < 0.1 || find_distance(2) < finish_distance) {
        refresh_servos();
        send_trig(0);
        send_trig(1);
        send_trig(2);
        distance_from_right_wall = find_distance(0);
        distance_from_left_wall = find_distance(1);
        pid_steer_imu(angle);
    }
    while(find_distance(2) > finish_distance) {
        refresh_servos();
        send_trig(0);
        send_trig(1);
        send_trig(2);
        distance_from_right_wall = find_distance(0);
        distance_from_left_wall = find_distance(1);
        pid_steer_imu(angle);
    }
    if (distance_from_right_wall > distance_from_left_wall) {
        direction = !direction;
    }
}

void read_pixy() {  //read the pixyCam values
  // grab blocks!
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0)
  {
    object_center_x = pixy.ccc.blocks[0].m_x;
    object_center_y = pixy.ccc.blocks[0].m_y;
    object_height = pixy.ccc.blocks[0].m_height;
    object_width = pixy.ccc.blocks[0].m_width;
    object_color = pixy.ccc.blocks[0].m_signature;
  }
}

int find_object() { //we forgot why we codded this, but it looks cute so we kept it :) (we acctually wrote it to avoid obstacle using only the pixyCam long story short it is not reliable)
    object_color = 0;
    read_pixy();
    return object_color;
}

void pass_the_corner(int position){ //passing the corner depends on the end position of pass the line (three casses right, left and middle)
    
    if (position == 0) {        //if the car ends in the right so the distace from outer-wall (!direction) shoud be close distance around 10
        follow_wall(65, 10, !direction);
        turn_90_short();
    } else if (position == 1) { //if the car ends in the left so the distace from outer-wall (!direction) shoud be far distance around 60
        send_trig(2);
        while (find_distance(2) < 0.1) {
            send_trig(2);
        }
        if (find_distance(2) > 100) {
            follow_wall(30, 42.5, !direction);
            turn_90_reverse();
        } else {
            follow_wall(30, 60, !direction);
            turn_90_reverse();
        }
    } else {                    //if the car ends in the middle so the distace from outer-wall (!direction) shoud be middle distance around 42.5
        follow_wall(30, 42.5, !direction);
        turn_90_reverse();
    }
    send_trig(3);
    // all functions before ends in the middle of the corner section but various distances from the back wall
    while (find_distance(3) > 18 || find_distance(3) < 0.1){ //drive back until backward distance becomes less than 18 cm
        speed_of_car = low_speed;
        pid_reverse_imu(0); 
        send_trig(3);
    }
    steer(0);
    move_car(0, 0);
    clean_all_measured_distance_arrays();
    delay(100);
}

int pass_the_section(){ 
    // right_counter = 0;
    refresh_servos();
    move_car(true, low_speed);
    int no_of_obstacles = 0;            //counting the obstacles in the section
    send_trig(3);
    find_distance(3);
    while (find_object() == 0){         //move by the wall until detecting an obstacles
        send_trig(3);
        find_distance(3);
        follow_wall(middle_distance, !direction);
    }
    if (find_distance(3) < first_obstacle_distance) {       //if the first obstacle detected is acttually in the 2 first obstacle places
                                                            //(there are 6 diffrent places for obstacles 2 close, 2 in the middle and 2 far)
        if (find_object() == 1) color_of_object = 1;        //if object is green set color_of_object to 1 if red to -1
        else color_of_object = -1;
        avoid_object_ultrasonic();
        no_of_obstacles++;
        send_trig(direction);
        double temp_find_direction = find_distance(direction);
        while (temp_find_direction < 0.1) {
            send_trig(direction);
            temp_find_direction = find_distance(direction);
        }
        while (find_object() == 0 && (temp_find_direction > 0.1 && temp_find_direction < 80)) { //keep following the outer wall until you detect an object or the inside wall disappers
            follow_wall(middle_distance, !direction);
            send_trig(direction);
            temp_find_direction = find_distance(direction);
        }
        if (temp_find_direction > 0.1 && temp_find_direction < 80 ) { //if the inside wall didn't dissapear yet there must be an obstacle ahead of you
            if (find_object() == 1) color_of_object = 1;
            else color_of_object = -1;
            avoid_object_ultrasonic();
            no_of_obstacles++;
            if (color_of_object == 1) {
                return 1;
            } else {
                return 0;
            }
        } else {
            return 2;
        }
    } else {
        if (find_object() == 1) color_of_object = 1; //the first obstacle detected is not in the 2 first obstacle places so there must be only one (middle or far)
        else color_of_object = -1;
        avoid_object_ultrasonic();
        no_of_obstacles++;
        if (color_of_object == 1) {
            return 1;
        } else {
            return 0;
        }
    }
}

void avoid_object_encoder() {                       //avoid obstacle based on encoder
    if (object_color == 1) {
        pid_steer_imu_encoder(avoid_start_distance_g, 0);
        pid_steer_imu_encoder(avoid_diagonal_distance_1_g, -1 * color_of_object * angle_of_avoid);
        pid_steer_imu_encoder(avoid_middle_distance_g, 0);
        pid_steer_imu_encoder(avoid_diagonal_distance_2_g, color_of_object * angle_of_avoid);
        pid_steer_imu_encoder(avoid_finish_distance_g, 0);
    } else {
        pid_steer_imu_encoder(avoid_start_distance_r, 0);
        pid_steer_imu_encoder(avoid_diagonal_distance_1_r, -1 * color_of_object * angle_of_avoid);
        pid_steer_imu_encoder(avoid_middle_distance_r, 0);
        pid_steer_imu_encoder(avoid_diagonal_distance_2_r, color_of_object * angle_of_avoid);
        pid_steer_imu_encoder(avoid_finish_distance_r, 0);
    }
}

void avoid_object_ultrasonic() {                    //avoid obstacle based on ultrasonic
    if (object_color == 1){                         //green obstacles solving 
        if (!direction) {
            send_trig(0);
                // or operator tests the first condition first and then tests the second
                // so that cause us a problem in doing functions that return a value
            double temp_find_distance_0 = find_distance(0);
            while (temp_find_distance_0 > distance_from_object || temp_find_distance_0 < 0.1){ //wait for the obstacle you have just detected
                follow_wall(10, 1);                 // drive near the left wall
                send_trig(0);
                temp_find_distance_0 = find_distance(0);
            }
            steer(-1 * imu_current_val + 25);       // move right for a litle time
            refresh_servos();
            delay(avoid_object_ultrasonic_delay);
        } else {
            send_trig(0);
            double temp_find_distance_0 = find_distance(0);
            while (temp_find_distance_0 > distance_from_object){
                follow_wall(75, 0);                 // drive near the left wall
                temp_find_distance_0 = find_distance(0);
                send_trig(0);
            }
            steer(-1 * imu_current_val + 25);       // move right for a litle time 
            refresh_servos();
            delay(avoid_object_ultrasonic_delay);
        }
    } else if (object_color == 2){                  //red obstacles solving 
        if (direction) {
            send_trig(1);
            double temp_find_distance_1 = find_distance(1);
            while (temp_find_distance_1 > distance_from_object || temp_find_distance_1 < 0.1){
                follow_wall(10, 0);                 // drive near the right wall
                send_trig(1);
                temp_find_distance_1 = find_distance(1);
            }
            steer(-1 * imu_current_val - 25);       // move left for a litle time
            refresh_servos();
            delay(avoid_object_ultrasonic_delay);
        } else {
            send_trig(1);
            double temp_find_distance_1 = find_distance(1);
            while (temp_find_distance_1 > distance_from_object){
                follow_wall(75, 1);                 // drive near the right wall
                temp_find_distance_1 = find_distance(1);
                send_trig(1);
            }
            steer(-1 * imu_current_val - 25);       // move left for a litle time
            refresh_servos();
            delay(avoid_object_ultrasonic_delay);
        }
    }
}