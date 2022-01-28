#pragma once

#include "util/vectorbase.h"
#include "util/quaternion.h"
#include "Simulator.h"

using namespace std;
using namespace GamePhysics;

class Point
{
public:
	Point(Vec3 p, Vec3 v, Vec3 f, float m, bool fixed, float c);
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

	//int isInPoint(Vec3 position);
	Mat4 Obj2WorldMatrix();
	float getC();
	void setFixed();

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	bool fixed;

	float c;		// bounciness coefficient
};
