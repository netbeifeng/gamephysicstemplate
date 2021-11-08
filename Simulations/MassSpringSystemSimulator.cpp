#include "MassSpringSystemSimulator.h"
#include "MassPoint.h"
#include "Spring.h"

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

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase)
	{
		case 0:
		{
			cout << "Demo 1 selected.\n\n";

			// Setup:
			Vec3 p0 = Vec3(0, 0, 0);
			Vec3 v0 = Vec3(-1, 0, 0);
			Vec3 p1 = Vec3(0, 2, 0);
			Vec3 v1 = Vec3(1, 0, 0);
			float m0 = 10, m1 = 10;

			Point pt0 = Point(p0, v0, m0);
			Point pt1 = Point(p1, v1, m1);

			Spring s = Spring(40, 1, pt0, pt1);

			cout << "Euler: \n";
			Spring s1 = s.makeEulerStep(0.1);
			cout << "position p1: " << s1.getP1().getPosition() << "\n";
			cout << "position p2: " << s1.getP2().getPosition() << "\n\n";


			cout << "Midpoint: \n";

			Spring sm = s.makeEulerStep(0.05);
			cout << "position p1: " << sm.getP1().getPosition() << "\n";
			cout << "position p2: " << sm.getP2().getPosition() << "\n";
			cout << "velocity p1: " << sm.getP1().getVelocity() << "\n";
			cout << "velocity p2: " << sm.getP2().getVelocity() << "\n";

			pt0.setVelocity(sm.getP1().getVelocity());
			pt1.setVelocity(sm.getP2().getVelocity());

			s = Spring(40, 1, pt0, pt1);
			s1 = s.makeEulerStep(0.1);
			cout << "position p1: " << s1.getP1().getPosition() << "\n";
			cout << "position p2: " << s1.getP2().getPosition() << "\n\n";
		}
		break;
		default:
		break;
	}
}

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
