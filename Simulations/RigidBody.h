#ifndef RIGIDBODY_h
#define RIGIDBODY_h

#include "util/vectorbase.h"
#include "util/quaternion.h"
#include "Simulator.h"
// include Simulator.h to use vector, weird

using namespace std;
using namespace GamePhysics;

class RigidBody {
public:
	// Construtors
	RigidBody(float b_mass, Vec3 b_size, Vec3 x_cm);

	// Functions
	Mat4 Obj2WorldMatrix();
	void setOrientation(Quat r);
	void setVelocity(Vec3 v_cm);
	void setForce(Vec3 force, Vec3 position);
	void integrate(float timestep);
	void getWorldInfoOfPoint(Vec3 position);

	Vec3 getVcm();
	float getMass();
	Mat4 getI();
	Vec3 getXcm();
	void setImpulse(float J, Vec3 x_i, Vec3 n);
	Vec3 getW();

private:
	// Attributes
	float b_mass;		// Body's mass
	Vec3 b_size;		// Body's size: width, height, depth
	Vec3 x_cm;	// Center Position
	Vec3 v_cm;	// Center Velocity
	Quat r;		// Orientation
	Vec3 L;		// Angular Momentum
	Vec3 w;		// Angular Velocity
	Mat4 I;		// Inverse of Inertia Tensor
	Vec3 q;		// Torque
	Vec3 f;		// Force: need total force to computer v_cm
};

#endif
