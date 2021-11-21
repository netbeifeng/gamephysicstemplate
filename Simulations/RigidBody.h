#pragma once

#include "util/vectorbase.h"
#include <vector>

using namespace std;
using namespace GamePhysics;


class RigidBody
{
public:
	RigidBody();
	RigidBody(float,float,float, float);

private:
	void construct(float, float, float, float);

	vector<Vec3> edges;
	vector<float> masses;
	Vec3 centerOfMass;
};
