#include "RigidBodySystemSimulator.h"
#include <math.h>
#define _USE_MATH_DEFINES

#define DEG2RAD (M_PI * 2)/360.f

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	
	m_rigidBodyList.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for each (RigidBody * rb in m_rigidBodyList) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
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
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0.0f, 0.0f, 90.0f * DEG2RAD));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	}
	case 1: {
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0.0f, 0.0f, 90.0f * DEG2RAD));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		break;
	}
	case 2: {
		break; 
	}
	case 3:
	{
	
		break;
	}
	default:
		break;
	}
	// END Setup Scene //

	// ** Write out to std output ** //
	switch (testCase)
	{
	case 0:
	{
		cout << "Demo 1 selected.\n\n";

		integrateAll(2);

		std::cout << "Position  (-0.3, -0.5, -0.25)" << std::endl;

		Vec3 pos = Vec3(-0.3, -0.5, -0.25);
		Vec3 linearVelocity = getLinearVelocityOfRigidBody(0);
		Vec3 angularVelocity = getAngularVelocityOfRigidBody(0);

		std::cout << "Velocity: " << linearVelocity + cross(angularVelocity, pos) << std::endl;
		std::cout << "Linear Velocity: " << linearVelocity << std::endl;
		std::cout << "Angular Velocity: " << angularVelocity << std::endl;
	}
	break;

	case 1:
		cout << "Demo 2 selected.\n\n";
		break;

	case 2:
		cout << "Demo 3 selected.\n\n";
		break;

	case 3:
		cout << "Demo 4 selected.\n";
		break;
	default:
		break;
	}
}

/*
In Demo 4, allow the top left corner to be moved.
Based on the mouse movement during a click, add difference vector to the current position.
*/
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 3 || m_iTestCase == 4 || m_iTestCase == 5)
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
			// Vec3 curpos = m_points[0]->getPosition();
			// m_points[0]->setPosition(curpos + inputWorld);
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
	m_rigidBodyList.push_back(new RigidBody(position, size, mass)); // push a new rigid body pointer to the list
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		std::cout << "NO FORCE???" << force << std::endl;
		toBeUpdatedRB->applyForce(force);       // add force
		toBeUpdatedRB->applyForceLoc(loc);      // set force loc
	}
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	RigidBody* toBeUpdatedRB = getRigidBodyByIdx(i);
	if (toBeUpdatedRB != nullptr) {
		toBeUpdatedRB->setUpRotation(orientation);
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

