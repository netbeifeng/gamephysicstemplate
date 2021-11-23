#include "RigidBody.h"

RigidBody::RigidBody(float x, float y, float z, float mass)
{
	o_edges.clear();
	masses.clear();
	massSum = 0;

	for (int i = 1; i >= -1; i -= 2)
	{
		for (int j = 1; j >= -1; j -= 2)
		{
			for (int k = 1; k >= -1; k -= 2)
			{
				o_edges.push_back(Vec3(i * x / 2, j * y / 2, k * z / 2));
				masses.push_back(mass);
				massSum += mass;
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

	w_torque = Vec3(0, 0, 0);
	w_linForce = Vec3(0, 0, 0);
}

void RigidBody::addForce(Vec3 w_f, Vec3 w_pos)
{
	w_torque += cross(w_pos, w_f);
	w_linForce += w_f;
}

void RigidBody::integrate(float timestep)
{
	// * Linear Euler step * //
	// Integrate Position
	w_centerOfMass = w_centerOfMass + timestep * w_centerVelocity;
	// Integrate Velocity
	w_centerVelocity = w_centerVelocity + timestep * w_linForce / massSum;

	// * Angular Integration * //
	w_angularMomentum = w_angularMomentum + timestep* w_torque;
	// Rotation matrices
	Mat4 o_to_w = o_to_w_orientation.getRotMat();
	Mat4 w_to_o = o_to_w_orientation.getRotMat();
	w_to_o.transpose();
	Mat4 w_inertiaTensorInv = w_to_o * o_inertiaTensorInv * o_to_w;

	w_angularVelocity = w_inertiaTensorInv.transformVector(w_angularMomentum);

	// Clear forces
	w_torque = Vec3(0, 0, 0);
	w_linForce = Vec3(0, 0, 0);
}

Vec3 RigidBody::getPointPosition(size_t i)
{
	Mat4 o_to_w = o_to_w_orientation.getRotMat();
	return w_centerOfMass + o_to_w.transformVector(o_edges[i]);
}

Vec3 RigidBody::getPointVelocity(size_t i)
{
	Mat4 o_to_w = o_to_w_orientation.getRotMat();
	cout << w_centerVelocity << " + " << "cross(" << w_angularVelocity << ", " << o_to_w.transformVector(o_edges[i]) << ")";
	return w_centerVelocity + cross(w_angularVelocity, o_to_w.transformVector(o_edges[i]));
}