#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_externalForce = Vec3();
	m_mouse = {0,0};
	m_trackmouse = {0,0};
	m_oldtrackmouse = {0,0};

	cout << "Constr yay";
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "D1,D2";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	cout << "Test yay";

}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {}
void RigidBodySystemSimulator::simulateTimestep(float timeStep) {}
void RigidBodySystemSimulator::onClick(int x, int y) {}
void RigidBodySystemSimulator::onMouse(int x, int y) {}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies() { return 0; }
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return Vec3(); }
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { return Vec3(); }
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { return Vec3(); }
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {}
