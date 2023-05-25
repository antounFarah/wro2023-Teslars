#pragma once
class myIMU
{
public:
	virtual void init() = 0;
	virtual void getyaw(float& y) = 0;
};

