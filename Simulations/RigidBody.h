#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include "Simulator.h"
struct Point {
	Point(Vec3 pos, float m) {
		this->pos = pos;
		this->m = m;
	};
	Vec3 pos;
	float m;
};

class RigidBody {

public:
	RigidBody();
	RigidBody(Vec3 position, Vec3 size, float mass);

	Mat4 getToWorldMatrix();

	void computeWorldMatrix();
	void preCompute();
	void initialize();

	void setRotation(Quat rotation);
	void updateForce(Vec3 force);
	void updateForceLoc(Vec3 loc);
	void updateTorque(Vec3 force, Vec3 loc);

	Vec3 getCenterPosition();
	Vec3 getLinearVelocity();
	Vec3 getAngularVelocity();

	void updateCenterPosition(Vec3 pos);
	void updateLinearVelocity(Vec3 lv);
	void updateAngularVelocity(Vec3 av);
	void updateAngularMomentum(Vec3 am);

	void updateInetriaTensor(Mat4 it);

	void updateRotation(Quat r);

	void integrate(float timeStep);

	void clear();


private:
	Mat4 m_toWorld;

	Vec3 m_force;
	Vec3 m_forceLoc;
	Vec3 m_centerPosition;
	Vec3 m_size;

	Vec3 m_torque;

	Quat m_rotation;

	Vec3 m_angularVelocity;
	Vec3 m_linearVelocity;
	Mat4 m_inertiaTensor;

	Vec3 m_angularMomentum;

	float m_mass;

	bool m_isFixed = false;
};

#endif