#include "RigidBody.h"

RigidBody::RigidBody(Vec3 size, float mass)
{
	_size = size;
	_mass = mass;

	w_centerOfMass = Vec3(0, 0, 0);
	centerVelocity = Vec3(0, 0, 0);
	orientation = Quat(0,0,0,1);

	float x = size.x;
	float y = size.y;
	float z = size.z;
	float mx = mass * (x / 2) * (x / 2);
	float my = mass * (y / 2) * (y / 2);
	float mz = mass * (z / 2) * (z / 2);
	
	inertiaTensorInv0 = Mat4(1/(my + mz), 0,    0   , 0,
								0, 1/(mx + mz), 0   , 0,
								0,    0, 1/(mx + my), 0,
								0,    0,    0   , 0);

	torque = Vec3(0, 0, 0);
	linForce = Vec3(0, 0, 0);
}

void RigidBody::addForce(Vec3 w_f, Vec3 w_pos)
{
	torque += cross(w_pos - w_centerOfMass, w_f);
	linForce += w_f;
}

Mat4 RigidBody::getWorldMatrix() {
	Mat4 translate, scale;

	translate.initTranslation(w_centerOfMass.x, w_centerOfMass.y, w_centerOfMass.z);
	scale.initScaling(_size.x, _size.y, _size.z);

	return scale * orientation.getRotMat() * translate;
}

Mat4 RigidBody::getWorldInvInertia()
{
	Mat4 rot = orientation.getRotMat();
	Mat4 rotT = orientation.getRotMat();
	rotT.transpose();
	return rot * inertiaTensorInv0 * rotT;
}

void RigidBody::applyImpulse(Vec3 position, float J, Vec3 colNormal)
{
	centerVelocity = centerVelocity + J * colNormal / _mass;
	angularMomentum = angularMomentum + cross(position, J * colNormal);
}

Vec3 RigidBody::getVelocityOf(Vec3 position)
{
	Mat4 rot = orientation.getRotMat();
	return centerVelocity + cross(angularVelocity, rot.transformVector(position));
}

void RigidBody::integrate(float timestep)
{
	// * Linear Euler step * //
	// Integrate Position
	w_centerOfMass = w_centerOfMass + timestep * centerVelocity;
	// Integrate Velocity
	centerVelocity = centerVelocity + timestep * linForce / _mass;

	// * Angular Integration * //
	// Integrate orientation (with normalization)
	orientation = orientation + timestep/2 * Quat(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0) * orientation;
	orientation /= orientation.norm();
	// Angular Momentum
	angularMomentum = angularMomentum + timestep* torque;

	// Rotation matrices
	angularVelocity = getWorldInvInertia().transformVector(angularMomentum);

	// Clear forces
	torque = Vec3(0, 0, 0);
	linForce = Vec3(0, 0, 0);
}

Vec3 RigidBody::getPointPosition(int i)
{
	Vec3 edge = Vec3(((i >> 2 & 1) * 2 - 1) * _size.x, ((i >> 1 & 1) * 2 - 1) * _size.y, ((i & 1) * 2 - 1) * _size.z)/2;
	Mat4 rot = orientation.getRotMat();
	return w_centerOfMass + rot.transformVector(edge);
}

Vec3 RigidBody::getPointVelocity(int i)
{
	Vec3 edge = Vec3(((i >> 2 & 1) * 2 - 1) * _size.x, ((i >> 1 & 1) * 2 - 1) * _size.y, ((i & 1) * 2 - 1) * _size.z) / 2;
	return getVelocityOf(edge);
}