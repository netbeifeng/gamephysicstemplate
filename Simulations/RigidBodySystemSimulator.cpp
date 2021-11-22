#include "RigidBodySystemSimulator.h"
#include "RigidBody.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_externalForce = Vec3();
	m_mouse = {0,0};
	m_trackmouse = {0,0};
	m_oldtrackmouse = {0,0};
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (testCase)
	{
	case 0:
	{
		RigidBody b = RigidBody(1, 0.6, 0.5, 2);
		cout << "RigidBody constructed.\n";
		cout << "Inverse object-space Inertia Tensor:\n" << b.getInvInertiaTensor() << "\n\n";
		b.setOrientation(Quat(0,0, -M_PI/2));

		b.addForce(Vec3(1,1,0), Vec3(0.3,0.5,0.25));
		cout << "Force added.\n";
		cout << "Torque: " << b.getTorque();
		break;
	}
	default:
		break;
	}
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
