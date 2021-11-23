#pragma once

#include "Simulator.h"
#include <vector>

using namespace std;
using namespace GamePhysics;

class RigidBody
{
public:
	RigidBody(float x=1,float y=1,float z=1, float m=1);

	void addForce(Vec3 w_f, Vec3 w_pos);
	void integrate(float);

	Vec3 getPointPosition(size_t i);
	Vec3 getPointVelocity(size_t i);
	Vec3 getTorque() { return w_torque; }
	Mat4 getInvInertiaTensor() { return o_inertiaTensorInv; }
	Vec3 getAngularMomentum() { return w_angularMomentum; }
	Vec3 getAngularVelocity() { return w_angularVelocity; }
	Vec3 getLinearVelocity() { return w_centerVelocity; }

	void setOrientation(Quat r) { o_to_w_orientation = r; }

private:
	vector<Vec3> o_edges;
	vector<float> masses;
	float massSum;
	Vec3 w_centerOfMass;
	Vec3 w_centerVelocity;
	Quat o_to_w_orientation;
	Vec3 w_angularMomentum;
	Vec3 w_angularVelocity;
	Mat4 o_inertiaTensorInv;
	Vec3 w_torque;
	Vec3 w_linForce;
};
