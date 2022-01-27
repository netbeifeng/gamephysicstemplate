#include "MassPoint.h"

Point::Point(Vec3 p, Vec3 v, Vec3 f, float m, bool b, float c, float radius)
{
	position = p;
	velocity = v;
	force = f;
	mass = m;
	fixed = b;

	this->c = c;
	this->radius = radius;
	L = Vec3(0, 0, 0);
	w = Vec3(0, 0, 0);
	q = Vec3(0, 0, 0);
	// height = 2 * radius
	I = Mat4(1 / (2 * m * radius * radius), 0, 0, 0,
		0, 1 / (2 * m * radius * radius), 0, 0,
		0, 0, 1 / (2 * m * radius * radius), 0,
		0, 0, 0, 0);
}

void Point::setPosition(Vec3 p) { position = p; }
void Point::setVelocity(Vec3 v) { velocity = v; }

Vec3 Point::getPosition() { return position; }
Vec3 Point::getVelocity() { return velocity; }
Vec3 Point::getForce() { return force; }

void Point::clearForce() {
	if (!fixed) {
		force = Vec3(0, 0, 0);
		q = Vec3(0, 0, 0);
	}
}

void Point::addForce(Vec3 f, Vec3 p)
{
	if (!fixed) {
		force += f;

		Vec3 x_i = (p.x-position.x, p.y-position.y, p.z-position.z);
		q = q + cross(x_i, f);	// x_i = x_world - x_cm
	}
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
	return new Point(pos, vel, Vec3(0,0,0), mass, fixed, 0.1, 0.05);
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
		L = L + timestep * q;
		w = I * L;

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

void Point::initializeLeapFrog(float timestep) {
	if (!fixed)
	{
		// Only update velocity at half the step size
		Vec3 accel = force / mass;
		velocity = velocity + timestep / 2 * accel;
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

/*
int Point::isInPoint(Vec3 p)
{
	int result = 0;
	float r = 0.1;
	if (p.x >= position.x - r && p.x <= position.x + r
		&& p.y >= position.y - r && p.y <= position.y + r) {
		result = 1;
	}
	return result;
}
*/

Mat4 Point::Obj2WorldMatrix()
{
	// not need rotMat because it is a sphere
	Mat4 scaleMat, translatMat, worldMatrix;
	scaleMat.initScaling(2*radius, 2 * radius, 2 * radius);
	translatMat.initTranslation(position.x, position.y, position.z);
	worldMatrix = scaleMat * translatMat;
	return worldMatrix;
}

float Point::getC()
{
	return c;
}

Vec3 Point::getW()
{
	return w;
}
Mat4 Point::getI()
{
	return I;
}
