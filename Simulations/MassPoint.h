#pragma once

#include "util/vectorbase.h"
using namespace GamePhysics;

class Point
{
public:
	Point(Vec3 p = Vec3(0, 0, 0), Vec3 v = Vec3(0, 0, 0), Vec3 f = Vec3(0, 0, 0), float m = 0, bool fixed = 0);
	void setPosition(Vec3);
	void setVelocity(Vec3);
	Vec3 getPosition();
	Vec3 getVelocity();
	Vec3 getForce();
	float getMass() { return mass; }

	void clearForce();
	void addForce(Vec3 f);
	void addAcceleration(Vec3 a);
	Point* integrated(float);
	void integrate(float);
	void integrateWithMidpoint(float, Point*);
	void initializeLeapFrog(float);
	void integrateLeapFrog(float);

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	bool fixed;
};
