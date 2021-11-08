#pragma once

#include "util/vectorbase.h"
using namespace GamePhysics;

class Point
{
public:
	Point(Vec3, Vec3, float);
	void setVelocity(Vec3);
	Vec3 getPosition();
	Vec3 getVelocity();
	Point applyForce(Vec3,float);

private:
	Vec3 position;
	Vec3 velocity;
	float mass;
};

Point::Point(Vec3 p = Vec3(0, 0, 0), Vec3 v = Vec3(0, 0, 0), float m = 0)
{
	position = p;
	velocity = v;
	mass	 = m;
}

void Point::setVelocity(Vec3 v) { velocity = v; }

Vec3 Point::getPosition() { return position; }
Vec3 Point::getVelocity() { return velocity; }

Point Point::applyForce(Vec3 f, float timestep)
{
	Vec3 accel = f / mass;
	Vec3 vel = velocity + timestep * accel;
	Vec3 pos = position + timestep * velocity;
	return Point(pos,vel,mass);
}

