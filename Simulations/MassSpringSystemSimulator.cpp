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
	return "Demo 1,Demo 2,Demo 3";
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

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	// Draw line for each spring
	DUC->beginLine();
	for each (Spring s in springs)
	{
		DUC->drawLine(s.getP1()->getPosition(), Vec3(1, 0, 0), s.getP2()->getPosition(), Vec3(0, 1, 0));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	// For Demos 1-3
	Point* pt0, *pt1;
	Spring s;
	float m0, m1;
	Vec3 p0, p1, v0, v1;

	// ** Setup Scene ** //
	switch (testCase)
	{
	// Setup for Demos 1-3 (Table 1.1)
	case 0: case 1: case 2:
		// Points
		p0 = Vec3(0, 0, 0);
		v0 = Vec3(-1, 0, 0);
		p1 = Vec3(0, 2, 0);
		v1 = Vec3(1, 0, 0);
		m0 = 10, m1 = 10;

		// The point objects:
		pt0 = new Point(p0, v0, Vec3(0, 0, 0), m0);
		pt1 = new Point(p1, v1, Vec3(0, 0, 0), m1);

		// The spring object:
		s = Spring(40, 1, pt0, pt1);

		// Push points onto the attribute
		for (auto p : points) { delete p; }
		points.clear();
		points.push_back(pt0);
		points.push_back(pt1);

		springs.clear();
		springs.push_back(s);

		break;

	default:
		break;
	}

	switch (testCase)
	{
		case 0:
		{
			cout << "Demo 1 selected.\n\n";

			cout << "Euler: \n";
			makeEulerStep(0.1);
			cout << "position p1: " << springs[0].getP1()->getPosition() << "\n";
			cout << "position p2: " << springs[0].getP2()->getPosition() << "\n\n";


			// Reset scene
			points[0] = pt0;
			points[1] = pt1;
			springs[0] = s;

			cout << "Midpoint: \n";
			makeMidpointStep(0.1);
			cout << "position p1: " << springs[0].getP1()->getPosition() << "\n";
			cout << "position p2: " << springs[0].getP2()->getPosition() << "\n\n";

			delete pt0, pt1;
		}
		break;

		case 1:
		{
			cout << "Demo 2 selected.\n\n";
		}
		break;

		case 2:
		{
			cout << "Demo 3 selected.\n\n";
		}
		break;

		default:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:
		makeEulerStep(0.005);
		break;
	case 2:
		makeMidpointStep(0.005);
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::makeEulerStep(float timeStep) {
	springs[0] = springs[0].makeEulerStep(timeStep);
	points[0] = springs[0].getP1();
	points[1] = springs[0].getP2();
}

void MassSpringSystemSimulator::makeMidpointStep(float timeStep) {
	Spring midPointSpring = springs[0].makeEulerStep(timeStep/2);
	points[0]->setVelocity(midPointSpring.getP1()->getVelocity());
	points[1]->setVelocity(midPointSpring.getP2()->getVelocity());
	springs[0] = Spring(springs[0].getStiffness(), springs[0].getRestLength(), points[0], points[1]);
	springs[0] = springs[0].makeEulerStep(timeStep);
	points[0] = springs[0].getP1();
	points[1] = springs[0].getP2();
}

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
