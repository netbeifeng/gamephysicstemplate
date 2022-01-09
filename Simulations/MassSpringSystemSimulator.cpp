#include "MassSpringSystemSimulator.h"
#include "MassPoint.h"
#include "Spring.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setMass(2);
	setStiffness(10000);
	setDampingFactor(0.8);

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 4 (Midpoint),Demo 5,Vertical";
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

	float grey = 0.5;
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.1, Vec3(grey, grey, grey));
	DUC->drawSphere(sphere.getPosition(), sphere.getRadius());
}

/*
Reacts to change in test case (e.g. the drop-down menu, or when "reset scene" is pressed).
- Sets up scene, and
- prints out a short message to standard output.
*/
void MassSpringSystemSimulator::notifyCaseChanged(int testCase, float timestep)
{
	// Pass argument to the attribute.
	m_iTestCase = testCase;

	Spring s;
	Vec3 p0;

	// ** Setup Scene ** //
	switch (testCase)
	{
	// Setup for Demos 4 and 5
	case 0: case 1:
	{
		sphere = RigidBodySphere(Vec3(0, 0, 0.5), 0.1, 20);
		sphere.addAcceleration(Vec3(0, 0, -1000));

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
				// Push point onto the attribute, with a small upward velocity, and no force;
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
	case 2:
	{
		sphere = RigidBodySphere(Vec3(0, 2, 0), 0.1, 20);

		// Construct a "sheet" of points connected with springs,
		// with fixed corners.

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
				// j running from left=-0.5 to right
				p0 = Vec3(-0.5 + j * stepSize, 0, 0.5 - i * stepSize);
				// Push point onto the attribute, with a small upward velocity, and no force;
				// fix the top corners.
				points.push_back(new Point(p0, Vec3(0, 0, 0), Vec3(0, 0, 0), m_fMass, (i == 0 || i + 1 == height) && (j == 0 || j + 1 == width)));
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
	// * For Demo 5 (leap-frog), initialize
	switch (testCase)
	{
		case 0:
			cout << "Demo 4 (Midpoint) selected.\n\n";
			break;

		case 1:
			cout << "Demo 5 selected.\n";
			cout << "Initialized leap-frog assuming a timestep of: " << timestep << ".\n\n";
			// Initialize velocities
		case 2:
			applyForcesToCurrentPoints(Vec3(0, -9, 0));
			for (Point* p : points) {
				p->initializeLeapFrog(timestep);
			}
			break;
		default:
			break;
	}
}

/*
In Demos 4 and 5, allow the top left corner to be moved.
Based on the mouse movement during a click, add the resulting difference vector to the current position.
*/
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
		makeMidpointStep(timeStep, Vec3(0, -9, 0));
		break;
	case 1: case 2:
		makeLeapFrogStep(timeStep, Vec3(0, -9, 0));
		break;
	default:
		break;
	}
	sphere.integrate(timeStep);

	// Collision detection
	for (Point* p : points)
	{
		Vec3 direction = p->getPosition() - sphere.getPosition();
		Vec3 distance = norm(direction);
		Vec3 normal = direction / distance;

		Vec3 rel_vel = p->getVelocity() - sphere.getVelocity();
		if (distance <= sphere.getRadius())
		{
			float J = -(1 + sphere.getBounciness()) * dot(rel_vel, normal) / (1/sphere.getMass() + 1/p->getMass());
			p->applyImpulse(J, normal);
			sphere.applyImpulse(J, -normal);
		}
	}
	enforceFloorBoundary();
}

void MassSpringSystemSimulator::applyForcesToCurrentPoints(Vec3 gravity)
{
	// Gravity:
	for (Point* p : points)
	{
		p->addAcceleration(gravity);
	}
	sphere.addAcceleration(gravity);
	// Internal forces:
	for (Spring s : springs)
	{
		s.applyElasticForceToPoints(points);
	}
}

void MassSpringSystemSimulator::makeMidpointStep(float timeStep, Vec3 gravity) {
	applyForcesToCurrentPoints(gravity);

	// Integration to midpoint
	vector<Point*> midpoints;
	midpoints.clear();
	for (Point* p : points)
	{
		midpoints.push_back(p->integrated(timeStep / 2));
	}

	// Forces at midpoint
	// Gravity:
	for (Point* p : midpoints)
	{
		p->addAcceleration(gravity);
	}
	// Internal forces:
	for (Spring s : springs)
	{
		s.applyElasticForceToPoints(midpoints);
	}

	// Calculate result
	for (size_t i = 0; i < points.size(); i++)
	{
		points[i]->integrateWithMidpoint(timeStep, midpoints[i]);
	}
	for (auto p : midpoints) { delete p; }
}

void MassSpringSystemSimulator::makeLeapFrogStep(float timeStep, Vec3 gravity)
{
	applyForcesToCurrentPoints(gravity);

	// Integration
	for (Point * p : points)
	{
		p->integrateLeapFrog(timeStep);
	}
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

	if (sphere.getPosition().y - sphere.getRadius() < -0.5)
	{
		Vec3 n = Vec3(0, 1, 0);
		float J = -(1 + sphere.getBounciness()) * dot(sphere.getVelocity(), n) * sphere.getMass();
		sphere.applyImpulse(J, n);
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
