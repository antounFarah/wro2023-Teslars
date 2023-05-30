#include "impu.h"

float yaw;
int mx = 41, my = 20;
impu imu(mx, my);

void setup(){
  imu.init();
  Serial.begin(115200);

}

void loop(){
    imu.getyaw(yaw);
    Serial.println(yaw);
}