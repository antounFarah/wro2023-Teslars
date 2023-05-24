#pragma once
#include "myIMU.h"


class myWMPU : public myIMU
{
	// Inherited via myIMU
	virtual void init() override;
	virtual void getyaw(float& y) override;
	void gessoffsets();
};

