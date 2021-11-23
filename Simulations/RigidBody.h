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

	Mat4 getWorldMatrix();
	Vec3 getCenterPosition() { return w_centerOfMass; };
	Vec3 getPointPosition(int i);
	Vec3 getPointVelocity(int i);
	Vec3 getTorque() { return w_torque; }
	Mat4 getInvInertiaTensor() { return o_inertiaTensorInv; }
	Vec3 getAngularMomentum() { return w_angularMomentum; }
	Vec3 getAngularVelocity() { return w_angularVelocity; }
	Vec3 getLinearVelocity() { return w_centerVelocity; }

	void setOrientation(Quat r) { o_to_w_orientation = r; }

private:
	float _mass;
	Vec3 _size;
	Vec3 w_centerOfMass;
	Vec3 w_centerVelocity;
	Quat o_to_w_orientation;
	Vec3 w_angularMomentum;
	Vec3 w_angularVelocity;
	Mat4 o_inertiaTensorInv;
	Vec3 w_torque;
	Vec3 w_linForce;
};
