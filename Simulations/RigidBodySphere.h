#pragma once
#include "util/vectorbase.h"
using namespace GamePhysics;
#include "MassPoint.h"

class RigidBodySphere
{
public:
	RigidBodySphere(Vec3 midpoint = Vec3(0,0,0), float radius = 0.1, float mass = 1, float bounciness = 0.9);

	// Gettters
	Vec3 getPosition() { return position; }
	float getRadius() { return radius; }
	float getMass() { return mass; }
	Vec3 getVelocity() { return velocity; }
	float getBounciness() { return bounciness; }

	void clearForce() { force = Vec3(0, 0, 0); }
	void addAcceleration(Vec3 a);
	void integrate(float);
	void applyImpulse(float, Vec3);
	void addForce(Vec3 f) { force += f; }
	void setPosition(Vec3 p) { position = p; }

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	float radius;
	float bounciness;
};

