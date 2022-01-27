#pragma once

#include "util/vectorbase.h"
#include "util/quaternion.h"
#include "Simulator.h"

using namespace std;
using namespace GamePhysics;

class Point
{
public:
	Point(Vec3 p, Vec3 v, Vec3 f, float m, bool fixed, float c, float radius);
	void setPosition(Vec3);
	void setVelocity(Vec3);
	Vec3 getPosition();
	Vec3 getVelocity();
	Vec3 getForce();
	float getMass() { return mass; }

	void clearForce();
	void addForce(Vec3 f, Vec3 p);
	void addAcceleration(Vec3 a);
	Point* integrated(float);
	void integrate(float);
	void integrateWithMidpoint(float, Point*);
	void initializeLeapFrog(float);
	void integrateLeapFrog(float);

	//int isInPoint(Vec3 position);
	Mat4 Obj2WorldMatrix();
	float getC();
	Vec3 getW();
	Mat4 getI();

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	bool fixed;

	float c;		// bounciness coefficient
	float radius;	// regard a point as a sphere
	Vec3 L;		// Angular Momentum
	Vec3 w;		// Angular Velocity
	Mat4 I;		// Inverse of Inertia Tensor
	Vec3 q;		// Torque
};
