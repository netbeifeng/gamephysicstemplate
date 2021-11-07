#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setMass(10);
	setStiffness(40);
	setDampingFactor(0);
	setIntegrator(0);

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {}
void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}




/// *** Specific functions *** ///

// Setters

void MassSpringSystemSimulator::setMass(float m) { m_fMass = m; }

void MassSpringSystemSimulator::setStiffness(float k) { m_fStiffness = k; }

void MassSpringSystemSimulator::setDampingFactor(float d) { m_fDamping = d; }


int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) { return 0; };
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {};

// Getters

int MassSpringSystemSimulator::getNumberOfMassPoints() { return 0; };
int MassSpringSystemSimulator::getNumberOfSprings() { return 0; };
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return Vec3(); };

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return Vec3(); };
void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {};
