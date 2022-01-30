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
	return "Vertical,Horizontal";
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

	// Draw sphere
	float grey = 0.5;
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.1, Vec3(grey, grey, grey));
	for (RigidBodySphere* s : spheres)
		DUC->drawSphere(s->getPosition(), s->getRadius());
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
	case 0:
	{
		for (auto s : spheres) { delete s; }
		spheres.clear();
		spheres.push_back(new RigidBodySphere(Vec3(0, 0, 0.5), 0.1, 20));
		spheres[0]->addAcceleration(Vec3(0, 0, -1000));

		// Construct a vertical "sheet" of points connected with springs,
		// with a small upward velocity
		// and two points at the top corners being fixed.

		// Rest length of the springs
		double restLen = 0.1;
		// Initial length of the springs
		double stepSize = 0.1;
		// Number of points: width*height
		int width = 12; int height = 12;

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
	case 1:
	{
		for (auto s : spheres) { delete s; }
		spheres.clear();
		spheres.push_back(new RigidBodySphere(Vec3(0, 2, 0), 0.1, 20));
		spheres.push_back(new RigidBodySphere(Vec3(0, 1, 0), 0.05, 5));
		spheres.push_back(new RigidBodySphere(Vec3(0, 3, 0), 0.3, 1));
		spheres.push_back(new RigidBodySphere(Vec3(0, 5, 0), 0.1, 99));

		// Construct a "sheet" of points connected with springs,
		// with fixed corners.

		// Rest length of the springs
		double restLen = 0.075;
		// Initial length of the springs
		double stepSize = 0.075;
		// Number of points: width*height
		int width = 12; int height = 12;

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

	// initialize leap-frog
	applyForcesToCurrentPoints(Vec3(0, -9, 0));
	for (Point* p : points)
		p->initializeLeapFrog(timestep);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 0 || m_iTestCase == 1)
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
	// Integration
	makeLeapFrogStep(timeStep, Vec3(0, -9, 0));
	for(RigidBodySphere* s : spheres)
		s->integrate(timeStep);

	// Collision detection
	for (RigidBodySphere* s : spheres)
	for (Point* p : points)
	{
		Vec3 direction = p->getPosition() - s->getPosition();
		Vec3 distance = norm(direction);
		Vec3 normal = direction / distance;

		Vec3 rel_vel = p->getVelocity() - s->getVelocity();
		if (distance <= s->getRadius())
		{
			float J = -(1 + s->getBounciness()) * dot(rel_vel, normal) / (1/s->getMass() + 1/p->getMass());
			p->applyImpulse(J, normal);
			s->applyImpulse(J, -normal);
			p->setPosition(p->getPosition() + timeStep * J * normal / p->getMass());
			s->setPosition(s->getPosition() - timeStep * J * normal / s->getMass());
			//p->setPosition(s->getPosition() + s->getRadius() * normal);
		}
	}
	enforceFloorBoundary();
}

void MassSpringSystemSimulator::applyForcesToCurrentPoints(Vec3 gravity)
{
	// Gravity:
	for (Point* p : points)
		p->addAcceleration(gravity);
	for (RigidBodySphere* s : spheres)
		s->addAcceleration(gravity);

	// Internal forces:
	for (Spring s : springs)
		s.applyElasticForceToPoints(points);
}

void MassSpringSystemSimulator::makeLeapFrogStep(float timeStep, Vec3 gravity)
{
	applyForcesToCurrentPoints(gravity);

	// Integration
	for (Point * p : points)
		p->integrateLeapFrog(timeStep);
}

void MassSpringSystemSimulator::enforceFloorBoundary()
{
	float floor = -1.0;

	// Points
	for each (Point* p in points)
	{
		Vec3 pos = p->getPosition();
		if (pos.y < floor) {
			p->setPosition(Vec3(pos.x, floor, pos.z));
		}
	}

	// Sphere
	for (RigidBodySphere* s : spheres)
	if (s->getPosition().y - s->getRadius() <= floor)
	{
		Vec3 n = Vec3(0, 1, 0);
		float J = -(1 + s->getBounciness()) * dot(s->getVelocity(), n) * s->getMass();
		s->applyImpulse(J, n);
		s->setPosition(Vec3(s->getPosition().x, floor + s->getRadius(), s->getPosition().z));
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
