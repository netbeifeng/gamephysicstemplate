#include "RigidBodySystemSimulator.h"
#include <math.h>
#include "collisionDetect.h"
#define _USE_MATH_DEFINES

#define DEG2RAD (M_PI * 2)/360.f

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
	std::cout << random(0., 1.) << std::endl;
	m_color = Vec3(random(0., 1.), random(0., 1.), random(0., 1.));
}

/// *** UI functions *** ///

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	// add UI box for tuning Bounciness
	if (m_iTestCase == 2) {
		TwAddVarRW(DUC->g_pTweakBar, "“Bounciness”", TW_TYPE_FLOAT, &m_Bounciness, "min=0.0 max=1.0 step=0.01");
	}
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	
	m_rigidBodyList.clear();
}

float RigidBodySystemSimulator::random(float a, float b) {
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {


	for each (RigidBody * rb in m_rigidBodyList) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, m_color);
		DUC->drawRigidBody(rb->getToWorldMatrix());
	}
}

/*
Reacts to change in test case (e.g. the drop-down menu, or when "reset scene" is pressed).
- Sets up scene, and
- prints out a short message to standard output.
*/
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	// Pass argument to the attribute.
	
	m_iTestCase = testCase;

	// ** Setup Scene ** //
	switch (testCase)
	{
	case 0: {
		cout << "Demo 1 selected.\n\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0, 0, M_PI / 2));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));

		integrateAll(2.f);

		std::cout << "Position  (-0.3, -0.5, -0.25) \n" << std::endl;

		Vec3 pos = Vec3(-0.3, -0.5, -0.25);
		Vec3 linearVelocity = getLinearVelocityOfRigidBody(0);
		Vec3 angularVelocity = getAngularVelocityOfRigidBody(0);

		std::cout << "Velocity: " << linearVelocity + cross(angularVelocity, pos) << std::endl;
		std::cout << "Linear Velocity: " << linearVelocity << std::endl;
		std::cout << "Angular Velocity: " << angularVelocity << std::endl;
		break;
	}
	case 1: {
		cout << "Demo 2 selected.\n\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0, 0, M_PI / 2));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	}
	case 2: {
		cout << "Demo 3 selected.\n\n";
		addRigidBody(Vec3(-0.25, -0.5, 0), Vec3(0.5, 0.5, 0.5), 2);
		addRigidBody(Vec3(0.25, 0.5, 0), Vec3(0.5, 0.5, 0.5), 2);

		setOrientationOf(0, Quat(0, 0, 0));
		setOrientationOf(1, Quat(0, 0, 0));

		applyForceOnBody(0, getRigidBodyByIdx(0)->getCenterPosition(), Vec3(0.3f, 0.5f, 0));
		applyForceOnBody(1, getRigidBodyByIdx(1)->getCenterPosition(), Vec3(-0.3f, -0.5f, 0));

		break;
	}
	case 3:
	{
		addRigidBody(Vec3(-0.25, -0.5, 0), Vec3(0.5, 0.5, 0.5), 2);
		addRigidBody(Vec3(-0.25, -0.5, 0), Vec3(0.5, 0.5, 0.5), 2);
		addRigidBody(Vec3(-0.25, -0.5, 0), Vec3(0.5, 0.5, 0.5), 2);
		addRigidBody(Vec3(-0.25, -0.5, 0), Vec3(0.5, 0.5, 0.5), 2);

		cout << "Demo 4 selected.\n\n";
		break;
	}
	default:
		break;
	}
	// END Setup Scene //
}

/*
In Demo 4, allow the top left corner to be moved.
Based on the mouse movement during a click, add difference vector to the current position.
*/
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 1)
	{
		// Apply the mouse deltas to point[0] (move along cameras view plane)
		Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
			// find a proper scale!
			float inputScale = 0.000015f;
			inputWorld = inputWorld * inputScale;
			// Apply difference vector

			// TODO: Add some interactions
			m_rigidBodyList[0]->applyForce(Vec3(0, 0, 0.1));
			m_rigidBodyList[0]->applyForceLoc(Vec3(0, 0, 0));
		}
	}

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		integrateAll(0.01f);
		break;
	case 2:
		integrateAll(0.02f);
		checkCollision();
		break;
	case 3:

		break;
	case 4:

		break;
	case 5:

		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_rigidBodyList.push_back(new RigidBody(position, size, mass)); 
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		std::cout << "Indicator: " << i << ", " << loc << ", " << force;
		toBeUpdatedRB->applyForce(force);       // add force
		toBeUpdatedRB->applyForceLoc(loc);      // set force loc
	}
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		toBeUpdatedRB->setUpRotation(orientation);
		toBeUpdatedRB->computeWorldMatrix();
	}
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_rigidBodyList.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getCenterPosition();
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getAngularVelocity();
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	RigidBody* fetchedRB = getRigidBodyByIdx(i);
	if (fetchedRB != nullptr) {
		return fetchedRB->getLinearVelocity();
	}
	return Vec3();
}


RigidBody* RigidBodySystemSimulator::getRigidBodyByIdx(int idx)
{
	if (idx > m_rigidBodyList.size() || idx < 0) {
		std::cout << "Invalid index " << idx << ", out of range [ 0 , " << m_rigidBodyList.size() << " ]" << std::endl;
		return nullptr; // if not found, return null pointer and check in each concrete functions
	}
	return m_rigidBodyList[idx]; // else return the fetched pointer
}

void RigidBodySystemSimulator::integrateAll(float timeStep) {
	for each (RigidBody * rb in m_rigidBodyList) {
		rb->integrate(timeStep);
	}
}

// Not so efficient O(n^2)
void RigidBodySystemSimulator::checkCollision() {
	int rigidBodiesNumber = getNumberOfRigidBodies();
	for (int idx = 0; idx < rigidBodiesNumber; idx++) {
		RigidBody* rigidBody1 = getRigidBodyByIdx(idx);
		for (int idx_s = idx + 1; idx_s < rigidBodiesNumber; idx_s++) {
			RigidBody* rigidBody2 = getRigidBodyByIdx(idx_s);
			CollisionInfo collisionInfo = checkCollisionSAT(rigidBody1->getToWorldMatrix(), rigidBody2->getToWorldMatrix());
			if (collisionInfo.isValid && (!rigidBody1->isFixed() || !rigidBody2->isFixed())) {
				Vec3 normal = collisionInfo.normalWorld;

				Vec3 collisionPositionRigidBody1 = collisionInfo.collisionPointWorld - rigidBody1->getCenterPosition();
				Vec3 collisionPositionRigidBody2 = collisionInfo.collisionPointWorld - rigidBody2->getCenterPosition();

				Vec3 collisionVelocityRigidBody1 = rigidBody1->getLinearVelocity() + cross(rigidBody1->getAngularVelocity(), collisionPositionRigidBody1);
				Vec3 collisionVelocityRigidBody2 = rigidBody2->getLinearVelocity() + cross(rigidBody2->getAngularVelocity(), collisionPositionRigidBody2);
				
				// Refer to Slide 32 - RigidBody 2D => 
				// https://www.moodle.tum.de/pluginfile.php/3399652/mod_resource/content/0/lecture03-rigid-bodies-2D.pdf
				
				float relativeVelocityDotNormal = dot(normal, (collisionVelocityRigidBody1 - collisionVelocityRigidBody2));
				
				Vec3 inertiaCrossRigidBody1 = cross(rigidBody1->getInertiaTensorInv() * cross(collisionPositionRigidBody1, normal), collisionPositionRigidBody1);
				Vec3 inertiaCrossRigidBody2 = cross(rigidBody2->getInertiaTensorInv() * cross(collisionPositionRigidBody2, normal), collisionPositionRigidBody2);

				double impulseCalculationPart1 = - (1.f + m_Bounciness) * relativeVelocityDotNormal;
				
				double impulseCalculationPart2 = ((1.f / rigidBody1->getMass()) + (1.f / rigidBody2->getMass()));
				
				Vec3 impulseCalculationPart3RigidBody1 = cross((rigidBody1->getInertiaTensorInv() * cross(collisionPositionRigidBody1, normal)), collisionPositionRigidBody1);
				Vec3 impulseCalculationPart3RigidBody2 = cross((rigidBody2->getInertiaTensorInv() * cross(collisionPositionRigidBody2, normal)), collisionPositionRigidBody2);
				
				double impulseCalculationPart3 = dot((impulseCalculationPart3RigidBody1 + impulseCalculationPart3RigidBody2), normal);
				
				double impulse = impulseCalculationPart1 / (impulseCalculationPart2 + impulseCalculationPart3);

				std::cout << "Impulse: " << impulse << std::endl;
				if (rigidBody1->isFixed()) { // only update RigidBody2
					rigidBody2->updateLinearVelocity((impulse * normal) / rigidBody2->getMass());
					rigidBody2->updateAngularMomentum(cross(collisionPositionRigidBody2, impulse * normal));
				} else if (rigidBody2->isFixed()) { // only update RigidBody1
					rigidBody1->updateLinearVelocity((impulse * normal) / rigidBody1->getMass());
					rigidBody1->updateAngularMomentum(cross(collisionPositionRigidBody1, impulse * normal));
				}

				// update both
				rigidBody1->updateLinearVelocity((impulse * normal) / rigidBody1->getMass());
				rigidBody1->updateAngularMomentum(cross(collisionPositionRigidBody1, impulse * normal));
				rigidBody2->updateLinearVelocity(-(impulse * normal) / rigidBody2->getMass());
				rigidBody2->updateAngularMomentum(-cross(collisionPositionRigidBody2, impulse * normal));
			}
		}
	}
}

// For interactions
// Picking Ray too complicated => Hard to implement
// Any Workaround?