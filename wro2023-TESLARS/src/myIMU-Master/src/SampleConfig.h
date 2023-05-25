#include "myIMU.h"
#include "myWMPU.h"

class SampleConfig
{
private:
	myWMPU mpu;
public:
	myIMU& getMPU();
};

