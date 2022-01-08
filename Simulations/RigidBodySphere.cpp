#include "RigidBodySphere.h"

RigidBodySphere::RigidBodySphere(Vec3 mid, float rad, float m, float bounce)
{
	position = mid;
	velocity = Vec3(0, 0, 0);
	force = Vec3(0, 0, 0);
	mass = m;
	radius = rad;
	bounciness = bounce;
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

void RigidBodySphere::applyImpulse(float J, Vec3 colNormal)
{
	velocity = velocity + J * colNormal / mass;
}

