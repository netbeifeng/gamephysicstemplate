#pragma once

#include "Simulator.h"
#include <vector>

using namespace std;
using namespace GamePhysics;

class RigidBody
{
public:
	RigidBody(Vec3 size, float mass=1);

	void addForce(Vec3 w_f, Vec3 w_pos);
	void integrate(float);
	void applyImpulse(Vec3 position, float J, Vec3 colNormal);

	Mat4 getWorldInvInertia();
	Vec3 getVelocityOf(Vec3 position);

	Mat4 getWorldMatrix();
	Vec3 getCenterPosition() { return w_centerOfMass; };
	Vec3 getPointPosition(int i);
	Vec3 getPointVelocity(int i);
	Vec3 getTorque() { return torque; }
	Mat4 getInvInertiaTensor() { return inertiaTensorInv0; }
	Vec3 getAngularMomentum() { return angularMomentum; }
	Vec3 getAngularVelocity() { return angularVelocity; }
	Vec3 getLinearVelocity() { return centerVelocity; }
	float getMass() { return _mass; }

	void setOrientation(Quat r) { orientation = r; }
	void setPosition(Vec3 p) { w_centerOfMass = p; }

private:
	float _mass;
	Vec3 _size;
	Vec3 w_centerOfMass;
	Vec3 centerVelocity;
	Quat orientation;
	Vec3 angularMomentum;
	Vec3 angularVelocity;
	Mat4 inertiaTensorInv0;
	Vec3 torque;
	Vec3 linForce;
};
