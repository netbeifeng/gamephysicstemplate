#pragma once
#include "util/vectorbase.h"
using namespace GamePhysics;
#include "MassPoint.h"

class RigidBodySphere
{
public:
	RigidBodySphere(Vec3 midpoint = Vec3(0,0,0), float radius = 0.1);

	// Gettters
	Vec3 getPosition() { return position; }
	float getRadius() { return radius; }

	void clearForce() { force = Vec3(0, 0, 0); }
	void addAcceleration(Vec3 a);
	void integrate(float);

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	float radius;
};

