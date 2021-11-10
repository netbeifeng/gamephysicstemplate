#include "MassPoint.h"

Point::Point(Vec3 p, Vec3 v, float m)
{
	position = p;
	velocity = v;
	mass = m;
}

void Point::setVelocity(Vec3 v) { velocity = v; }

Vec3 Point::getPosition() { return position; }
Vec3 Point::getVelocity() { return velocity; }

/*
Returns a new point object with the old velocity applied to the position and the force f applied to the velocity after an Euler timestep. The current object remains unchanged.
**/
Point* Point::applyForce(Vec3 f, float timestep)
{
	Vec3 accel = f / mass;
	Vec3 vel = velocity + timestep * accel;
	Vec3 pos = position + timestep * velocity;
	return new Point(pos, vel, mass);
}
