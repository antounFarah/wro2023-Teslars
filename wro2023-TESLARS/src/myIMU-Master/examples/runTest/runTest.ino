#include "myIMU.h"
#include "SampleConfig.h"

SampleConfig configg;
myIMU& mpu = configg.getMPU();

float yaw;
// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	while (!Serial);
	mpu.init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	mpu.getyaw(yaw);
	Serial.println(yaw);
}
