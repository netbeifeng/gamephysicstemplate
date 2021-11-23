#include "RigidBody.h"

RigidBody::RigidBody(Vec3 size, float mass)
{
	_size = size;
	_mass = mass;

	w_centerOfMass = Vec3(0, 0, 0);
	w_centerVelocity = Vec3(0, 0, 0);
	o_to_w_orientation = Quat(0,0,0,1);

	float x = size.x;
	float y = size.y;
	float z = size.z;
	float mx = mass * (x / 2) * (x / 2);
	float my = mass * (y / 2) * (y / 2);
	float mz = mass * (z / 2) * (z / 2);
	
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

Mat4 RigidBody::getWorldMatrix() {
	Mat4 tranlate, scale;

	tranlate.initTranslation(w_centerOfMass.x, w_centerOfMass.y, w_centerOfMass.z);
	scale.initScaling(_size.x, _size.y, _size.z);

	return scale * tranlate * o_to_w_orientation.getRotMat();
}

void RigidBody::integrate(float timestep)
{
	// * Linear Euler step * //
	// Integrate Position
	w_centerOfMass = w_centerOfMass + timestep * w_centerVelocity;
	// Integrate Velocity
	w_centerVelocity = w_centerVelocity + timestep * w_linForce / _mass;

	// * Angular Integration * //
	// Rotation
	o_to_w_orientation = o_to_w_orientation + timestep/2 * Quat(w_angularVelocity.x, w_angularVelocity.y, w_angularVelocity.z, 0) * o_to_w_orientation;
	// Normalize
	o_to_w_orientation /= o_to_w_orientation.norm();
	// Angular Momentum
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

Vec3 RigidBody::getPointPosition(int i)
{
	Vec3 edge = Vec3(((i >> 2 & 1) * 2 - 1) * _size.x, ((i >> 1 & 1) * 2 - 1) * _size.y, ((i & 1) * 2 - 1) * _size.z)/2;
	Mat4 o_to_w = o_to_w_orientation.getRotMat();
	return w_centerOfMass + o_to_w.transformVector(edge);
}

Vec3 RigidBody::getPointVelocity(int i)
{
	Vec3 edge = Vec3(((i >> 2 & 1) * 2 - 1) * _size.x, ((i >> 1 & 1) * 2 - 1) * _size.y, ((i & 1) * 2 - 1) * _size.z) / 2;
	Mat4 o_to_w = o_to_w_orientation.getRotMat();
	return w_centerVelocity + cross(w_angularVelocity, o_to_w.transformVector(edge));
}