#include "MassPoint.h"

Point::Point(Vec3 p, Vec3 v, Vec3 f, float m, bool b)
{
	position = p;
	velocity = v;
	force = f;
	mass = m;
	fixed = b;
}

void Point::setPosition(Vec3 p) { position = p; }
void Point::setVelocity(Vec3 v) { velocity = v; }

Vec3 Point::getPosition() { return position; }
Vec3 Point::getVelocity() { return velocity; }
Vec3 Point::getForce() { return force; }

void Point::clearForce() {
	if (!fixed)
		force = Vec3(0, 0, 0);
}

void Point::addForce(Vec3 f)
{
	if (!fixed)
		force += f;
}

void Point::addAcceleration(Vec3 a)
{
	if (!fixed)
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
	if (!fixed)
	{
		position = position + timestep * velocity;

		Vec3 accel = force / mass;
		velocity = velocity + timestep * accel;
	}
}

void Point::integrateWithMidpoint(float timestep, Point* midpoint)
{
	if (!fixed)
	{
		position = position + timestep * (midpoint->getVelocity());

		Vec3 accel = midpoint->getForce() / mass;
		velocity = velocity + timestep * accel;
	}
}