#include "MassPoint.h"

Point::Point(Vec3 p, Vec3 v, Vec3 f, float m)
{
	position = p;
	velocity = v;
	force = f;
	mass = m;
}

void Point::setPosition(Vec3 p) { position = p; }
void Point::setVelocity(Vec3 v) { velocity = v; }

Vec3 Point::getPosition() { return position; }
Vec3 Point::getVelocity() { return velocity; }
Vec3 Point::getForce() { return force; }

void Point::clearForce() {
	force = Vec3(0, 0, 0);
}

void Point::addForce(Vec3 f)
{
	force += f;
}

void Point::addAcceleration(Vec3 a)
{
	force += a / mass;
}

Point* Point::integrated(float timestep)
{
	Vec3 accel = force / mass;
	Vec3 vel = velocity + timestep * accel;
	Vec3 pos = position + timestep * velocity;
	return new Point(position, vel, force, mass);
}

void Point::integrate(float timestep)
{
	position = position + timestep * velocity;

	Vec3 accel = force / mass;
	velocity = velocity + timestep * accel;
}

void Point::integrateWithMidpoint(float timestep, Point* midpoint)
{
	position = position + timestep * (midpoint->getVelocity());

	Vec3 accel = midpoint->getForce() / mass;
	velocity = velocity + timestep * accel;
}