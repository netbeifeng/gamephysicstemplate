#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_externalForce = Vec3();
	m_mouse = {0,0};
	m_trackmouse = {0,0};
	m_oldtrackmouse = {0,0};
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->drawRigidBody(bodies[0].getWorldMatrix());
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (testCase)
	{
	case 0:
	{
		bodies.clear();

		bodies.push_back(RigidBody(Vec3(1, 0.6, 0.5), 2));
		bodies[0].setOrientation(Quat(0,0, M_PI/2));
		// cout << "Set orientation to " << Quat(0, 0, M_PI/2) << "\n";

		bodies[0].addForce(Vec3(1,1,0), Vec3(0.3,0.5,0.25));
		// cout << "Force added.\n";
		// cout << "Torque: " << bodies[0].getTorque() << "\n";

		int pt = 3;
		cout << "Original point position: " << bodies[0].getPointPosition(pt) << "\n";
		bodies[0].integrate(2);
		cout << "Integrated total point velocity: " << bodies[0].getPointVelocity(pt) << "\n";
		cout << "Body Angular Velocity: " << bodies[0].getAngularVelocity() << "\n";
		cout << "Body Linear Velocity: " << bodies[0].getLinearVelocity() << "\n";

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
