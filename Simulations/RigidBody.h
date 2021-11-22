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

	vector<Vec3> edges;
	vector<float> masses;
	Vec3 centerOfMass;
};
