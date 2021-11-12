#include "Spring.h"

Spring::Spring(float k, float L, float d, size_t pt1, size_t pt2)
{
	stiffness = k;
	restLength = L;
	dampingFactor = d;
	p1 = pt1;
	p2 = pt2;
}

float Spring::getRestLength() { return restLength; }
float Spring::getStiffness() { return stiffness; }
Point* Spring::getP1(std::vector<Point*> points) { return points[p1]; }
Point* Spring::getP2(std::vector<Point*> points) { return points[p2]; }

Vec3 Spring::getHookeForce(std::vector<Point*> points)
{
	Point* pt1 = getP1(points);
	Point* pt2 = getP2(points);
	Vec3 distVect = pt1->getPosition() - pt2->getPosition();
	float currentLength = sqrt(distVect.squaredDistanceTo(Vec3(0, 0, 0)));
	Vec3 force = stiffness * (restLength - currentLength) * distVect / currentLength;
	return force;
}

void Spring::applyElasticForceToPoints(std::vector<Point*> points)
{
	Point* pt1 = getP1(points);
	Point* pt2 = getP2(points);
	Vec3 f1 = getHookeForce(points);
	Vec3 f2 = -f1;
	pt1->addForce(f1);
	pt2->addForce(f2);

	// Damping
	Vec3 dampingForceP1 = dampingFactor * pt1->getVelocity() * pt1->getMass();
	Vec3 dampingForceP2 = dampingFactor * pt2->getVelocity() * pt2->getMass();
	pt1->addForce(-dampingForceP1);
	pt2->addForce(-dampingForceP2);
}
