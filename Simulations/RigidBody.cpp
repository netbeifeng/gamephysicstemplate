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
	o_to_w_orientation = Quat(0,0,0,1);

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

void RigidBody::addForce(Vec3 w_f, Vec3 w_pos)
{
	Mat4 w_to_o = o_to_w_orientation.getRotMat();
	w_to_o.transpose();
	Vec3 o_f = w_to_o.transformVector(w_f);
	Vec3 o_pos = w_to_o.transformVector(w_pos);

	o_torque += cross(o_pos, o_f);
	o_linForce += o_f;
}

