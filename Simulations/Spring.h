#pragma once

#include "util/vectorbase.h"
using namespace GamePhysics;
#include "MassPoint.h"

class Spring
{
public:
	Spring(float k = 0, float L = 0, Point p1 = Point(), Point p2= Point());

	float getRestLength();
	float getStiffness();
	Point getP1();
	Point getP2();

	Vec3 getHookeForce();
	Spring makeEulerStep(float);

private:
	float restLength;
	float stiffness;
	Point p1;
	Point p2;
};
