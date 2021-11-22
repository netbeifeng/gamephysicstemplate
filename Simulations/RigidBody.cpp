#include "RigidBody.h"

RigidBody::RigidBody(float x, float y, float z, float mass)
{
	o_edges.clear();
	masses.clear();

	for (int i = 1; i > 0; i -= 2)
	{
		for (int j = 1; j > 0; j -= 2)
		{
			for (int k = 1; k > 0; k -= 2)
			{
				o_edges.push_back(Vec3(i * x / 2, j * y / 2, k * z / 2));
				masses.push_back(mass);
			}
		}
	}

	w_centerOfMass = Vec3(0, 0, 0);
	w_centerVelocity = Vec3(0, 0, 0);
	w_to_o_orientation = Quat(1,0,0,0);

	float mx = mass * 8 * (x / 2) * (x / 2);
	float my = mass * 8 * (y / 2) * (y / 2);
	float mz = mass * 8 * (z / 2) * (z / 2);
	
	o_inertiaTensorInv = Mat4(1/(my + mz), 0,    0   , 0,
								0, 1/(mx + mz), 0   , 0,
								0,    0, 1/(mx + my), 0,
								0,    0,    0   , 0);

	o_torque = Vec3(0, 0, 0);
	o_linForce = Vec3(0, 0, 0);
}
