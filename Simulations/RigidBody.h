#pragma once

#include "Simulator.h"
#include <vector>

using namespace std;
using namespace GamePhysics;


class RigidBody
{
public:
	RigidBody(float x=1,float y=1,float z=1, float m=1);


private:
	vector<Vec3> o_edges;
	vector<float> masses;
	Vec3 w_centerOfMass;
	Vec3 w_centerVelocity;
	Quat w_to_o_orientation;
	Vec3 angularMomentum;
	Mat4 o_inertiaTensorInv;
	Vec3 o_torque;
	Vec3 o_linForce;
};
