#include "MassSpringSystemSimulator.h"
#include "MassPoint.h"
#include "Spring.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setMass(10);
	setStiffness(10000);
	setDampingFactor(0.8);
	setIntegrator(0);

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4 (Euler),Demo 4 (Midpoint)";
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
		DUC->drawLine(s.getP1(points)->getPosition(), Vec3(1, 0, 0), s.getP2(points)->getPosition(), Vec3(0, 1, 0));
	}
	DUC->endLine();
}

/*
Reacts to change in test case (e.g. the drop-down menu, or when "reset scene" is pressed).
- Sets up scene, and
- prints out a short message to standard output.
*/
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	// Pass argument to the attribute.
	m_iTestCase = testCase;

	Point* pt0, *pt1;
	Spring s;
	float m0, m1;
	Vec3 p0, p1, v0, v1;

	// ** Setup Scene ** //
	switch (testCase)
	{
	// Setup for Demos 1-3 (Table 1.1)
	case 0: case 1: case 2:
		// * Points * //
		p0 = Vec3(0, 0, 0);
		v0 = Vec3(-1, 0, 0);
		p1 = Vec3(0, 2, 0);
		v1 = Vec3(1, 0, 0);
		m0 = 10, m1 = 10;

		// The point objects:
		pt0 = new Point(p0, v0, Vec3(0, 0, 0), m0, false);
		pt1 = new Point(p1, v1, Vec3(0, 0, 0), m1, false);

		// Push points onto the attribute
		for (auto p : points) { delete p; }
		points.clear();
		points.push_back(pt0);
		points.push_back(pt1);

		// * Spring * //
		// The spring object:
		s = Spring(40, 1, 0, 0, 1);

		// Push points onto the attribute
		springs.clear();
		springs.push_back(s);

		break;

	// Setup for Demo 4
	case 3: case 4:
	{
		// Construct a "sheet" of points connected with springs,
		// with a small upward velocity
		// and two points at the top corners being fixed.

		// Rest length of the springs
		double restLen = 0.1;
		// Initial length of the springs
		double stepSize = 0.1;
		// Number of points: width*height
		int width = 8; int height = 8;

		// * Points * //
		for (auto p : points) { delete p; }
		points.clear();
		for (double i = 0; i < height; i++)
		{
			for (double j = 0; j < width; j++)
			{
				// i running from from top=0.5 to bottom,
				// j running from left=-0.5 to right,
				// with a slight forward tilt of stepSize/10 per row
				p0 = Vec3(-0.5 + j * stepSize, 0.5 - i * stepSize, i * stepSize/10);
				// Push point onto the attribute, with a small upward velocity no force;
				// fix the top corners.
				points.push_back(new Point(p0, Vec3(0,1,0), Vec3(0,0,0), m_fMass, i == 0 && (j == 0 || j+1==width)));
			}
		}

		// * Springs * //
		springs.clear();
		// Horizontal connections
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				s = Spring(m_fStiffness, restLen, m_fDamping, i * width + j, i * width + j + 1);
				springs.push_back(s);
			}
		}
		// Vertical connections
		for (int j = 0; j < width; j++)
		{
			for (int i = 0; i < height - 1; i++)
			{
				s = Spring(m_fStiffness, restLen, m_fDamping, i * width + j, (i + 1) * width + j);
				springs.push_back(s);
			}
		}
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

			cout << "Euler: \n";
			makeEulerStep(0.1);
			cout << "position p1: " << springs[0].getP1(points)->getPosition() << "\n";
			cout << "position p2: " << springs[0].getP2(points)->getPosition() << "\n";
			cout << "velocity p1: " << springs[0].getP1(points)->getVelocity() << "\n";
			cout << "velocity p2: " << springs[0].getP2(points)->getVelocity() << "\n";
			cout << "\n";

			// Reset scene
			for (auto p : points) { delete p; }
			points.clear();
			pt0 = new Point(p0, v0, Vec3(0, 0, 0), m0, false);
			pt1 = new Point(p1, v1, Vec3(0, 0, 0), m1, false);
			points.push_back(pt0);
			points.push_back(pt1);

			cout << "Midpoint: \n";
			makeMidpointStep(0.1);
			cout << "position p1: " << springs[0].getP1(points)->getPosition() << "\n";
			cout << "position p2: " << springs[0].getP2(points)->getPosition() << "\n";
			cout << "velocity p1: " << springs[0].getP1(points)->getVelocity() << "\n";
			cout << "velocity p2: " << springs[0].getP2(points)->getVelocity() << "\n";
			cout << "\n";

		}
		break;

		case 1:
			cout << "Demo 2 selected.\n\n";
			break;

		case 2:
			cout << "Demo 3 selected.\n\n";
			break;

		case 3:
			cout << "Demo 4 (Euler) selected.\n";
			break;

		case 4:
			cout << "Demo 4 (Midpoint) selected.\n\n";
			break;
		default:
			break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 3 || m_iTestCase == 4)
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
			Vec3 curpos = points[0]->getPosition();
			points[0]->setPosition(curpos + inputWorld);
		}
	}

}

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
	case 3:
		makeEulerStep(timeStep);
		enforceFloorBoundary();
		break;
	case 4:
		makeMidpointStep(timeStep);
		enforceFloorBoundary();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::makeEulerStep(float timeStep) {
	// Forces
	for (size_t i = 0; i < points.size(); i++)
	{
		// Gravity
		if (m_iTestCase == 3) {
			points[i]->addAcceleration(Vec3(0, -9, 0));
		}
	}
	for (size_t i = 0; i < springs.size(); i++)
	{
		springs[i].applyElasticForceToPoints(points);
	}

	// Integration
	for (size_t i = 0; i < points.size(); i++)
	{
		points[i]->integrate(timeStep);
	}
}

void MassSpringSystemSimulator::makeMidpointStep(float timeStep) {
	// Forces at current points
	for (size_t i = 0; i < points.size(); i++)
	{
		// Gravity
		if (m_iTestCase == 4) {
			points[i]->addAcceleration(Vec3(0, -9, 0));
		}
	}
	for (size_t i = 0; i < springs.size(); i++)
	{
		springs[i].applyElasticForceToPoints(points);
	}

	// Integration to midpoint
	vector<Point*> midpoints;
	midpoints.clear();
	for (size_t i = 0; i < points.size(); i++)
	{
		midpoints.push_back(points[i]->integrated(timeStep / 2));
	}

	// Forces at midpoint
	for (size_t i = 0; i < midpoints.size(); i++)
	{
		if (m_iTestCase == 4) {
			midpoints[i]->addAcceleration(Vec3(0, -9, 0));
		}
	}
	for (size_t i = 0; i < springs.size(); i++)
	{
		springs[i].applyElasticForceToPoints(midpoints);
	}

	// Calculate result
	for (size_t i = 0; i < points.size(); i++)
	{
		points[i]->integrateWithMidpoint(timeStep, midpoints[i]);
	}
	for (auto p : midpoints) { delete p; }
}

void MassSpringSystemSimulator::enforceFloorBoundary()
{
	for each (Point* p in points)
	{
		Vec3 pos = p->getPosition();
		if (pos.y < -0.5) {
			p->setPosition(Vec3(pos.x, -0.5, pos.z));
		}
	}
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

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return points[index]->getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return points[index]->getVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {};
