#include "RigidBodySphere.h"

RigidBodySphere::RigidBodySphere(Vec3 mid, float rad)
{
	position = mid;
	velocity = Vec3(0, 0, 0);
	force = Vec3(0, 0, 0);
	mass = 1;
	radius = rad;
}

void RigidBodySphere::addAcceleration(Vec3 a) {
	force += a * mass;
}

void RigidBodySphere::integrate(float timestep)
{
	position = position + timestep * velocity;

	Vec3 accel = force / mass;
	velocity = velocity + timestep * accel;

	clearForce();
}

