#include "Spring.h"

Spring::Spring(float k, float L, Point pt1, Point pt2)
{
	stiffness = k;
	restLength = L;
	p1 = pt1;
	p2 = pt2;
}

Point Spring::getP1() { return p1; }
Point Spring::getP2() { return p2; }

Vec3 Spring::getHookeForce()
{
	Vec3 distVect = p1.getPosition() - p2.getPosition();
	float currentLength = sqrt(distVect.squaredDistanceTo(Vec3(0, 0, 0)));
	Vec3 force = stiffness * (restLength - currentLength) * distVect / currentLength;
	return force;
}

Spring Spring::makeEulerStep(float timestep)
{
	Vec3 f1 = getHookeForce();
	Vec3 f2 = -f1;
	Point q1 = p1.applyForce(f1, timestep);
	Point q2 = p2.applyForce(f2, timestep);
	return Spring(stiffness, restLength, q1, q2);
}