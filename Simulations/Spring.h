#pragma once

#include <vector>

#include "util/vectorbase.h"
using namespace GamePhysics;
#include "MassPoint.h"

class Spring
{
public:
	Spring(float k = 0, float L = 0, int p1 = 0, int p2= 0);

	Point* getP1(std::vector<Point*> points);
	Point* getP2(std::vector<Point*> points);
	float getRestLength();
	float getStiffness();

	Vec3 getHookeForce(std::vector<Point*>);
	void applyElasticForceToPoints(std::vector<Point*>);

private:
	int p1;
	int p2;
	float restLength;
	float stiffness;
};
