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

/*
Converts acceleration to force and adds it.
*/
void Point::addAcceleration(Vec3 a)
{
	if (!fixed)
		force += a * mass;
}

/*
Returns a new Point resulting from integrating the current Point, without modifying it.
The same as applying integrate() to a clone of the current object.
(Used e.g. in the midpoint method).
*/
Point* Point::integrated(float timestep)
{
	Vec3 accel = force / mass;
	Vec3 vel = velocity + timestep * accel;
	Vec3 pos = position + timestep * velocity;
	return new Point(pos, vel, Vec3(0,0,0), mass, fixed);
}

/*
Applies integration on position and velocity, both based on the old values, and clears force.
*/
void Point::integrate(float timestep)
{
	if (!fixed)
	{
		position = position + timestep * velocity;

		Vec3 accel = force / mass;
		velocity = velocity + timestep * accel;
	}
	clearForce();
}

/*
Applies midpoint integration, with the supplied midpoint.
Position and velocity are updated based on the old values and the changes at the midpoint.
Clears force.
*/
void Point::integrateWithMidpoint(float timestep, Point* midpoint)
{
	if (!fixed)
	{
		position = position + timestep * (midpoint->getVelocity());

		Vec3 accel = midpoint->getForce() / mass;
		velocity = velocity + timestep * accel;
	}
	clearForce();
}

void Point::integrateLeapFrog(float timestep)
{
	// Assuming that the velocity is half a step ahead.
	if (!fixed)
	{
		// Normal Euler for velocity
		Vec3 accel = force / mass;
		velocity = velocity + timestep * accel;
		// Use updated velocity for the position
		position = position + timestep * velocity;
	}
	clearForce();
}