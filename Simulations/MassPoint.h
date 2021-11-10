#pragma once

#include "util/vectorbase.h"
using namespace GamePhysics;

class Point
{
public:
	Point(Vec3 p = Vec3(0, 0, 0), Vec3 v = Vec3(0, 0, 0), Vec3 f = Vec3(0, 0, 0), float m = 0);
	void setVelocity(Vec3);
	Vec3 getPosition();
	Vec3 getVelocity();
	Point* applyForce(Vec3,float);

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
};
