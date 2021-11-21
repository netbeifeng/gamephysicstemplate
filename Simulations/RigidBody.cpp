#include "RigidBody.h"

RigidBody::RigidBody()
{
	construct(1, 1, 1, 1);
}

RigidBody::RigidBody(float x, float y, float z, float mass)
{
	construct(x, y, z, mass);
}

void RigidBody::construct(float x, float y, float z, float mass)
{
	edges.clear();
	masses.clear();

	for (int i = 1; i > 0; i -= 2)
	{
		for (int j = 1; j > 0; j -= 2)
		{
			for (int k = 1; k > 0; k -= 2)
			{
				edges.push_back(Vec3(i * x / 2, j * y / 2, k * z / 2));
				edges.push_back(mass);
			}
		}
	}

	centerOfMass = Vec3(0, 0, 0);
}
