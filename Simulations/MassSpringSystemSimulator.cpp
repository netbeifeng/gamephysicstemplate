#include "MassSpringSystemSimulator.h"

// Construtors
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;	// inherit from Simulator
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = 0;	// refer to three cases, the same with m_iTestCase
	m_externalForce = Vec3();
}


// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "EULER,LEAPFROG,MIDPOINT";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;	// use this pointer because DUC is duplicate
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	m_iTestCase = m_iIntegrator;
	switch (m_iTestCase)
	{
	case 0:
		for (int i = 0; i < points.size(); i++) {
			DUC->drawSphere(points[i].position, Vec3(0.05, 0.05, 0.05));
		}
		for (int i = 0; i < springs.size(); i++) {
			DUC->beginLine();
			DUC->drawLine(points[springs[i].point0].position, (1, 1, 1), points[springs[i].point1].position, (1, 1, 1));
			DUC->endLine();
		}
		break;
	case 1:
		for (int i = 0; i < points.size(); i++) {
			DUC->drawSphere(points[i].position, Vec3(0.05, 0.05, 0.05));
		}
		for (int i = 0; i < springs.size(); i++) {
			DUC->beginLine();
			DUC->drawLine(points[springs[i].point0].position, (1, 1, 1), points[springs[i].point1].position, (1, 1, 1));
			DUC->endLine();
		}
		break;
	case 2:
		for (int i = 0; i < points.size(); i++) {
			DUC->drawSphere(points[i].position, Vec3(0.05, 0.05, 0.05));
		}
		for (int i = 0; i < springs.size(); i++) {
			DUC->beginLine();
			DUC->drawLine(points[springs[i].point0].position, (1, 1, 1), points[springs[i].point1].position, (1, 1, 1));
			DUC->endLine();
		}
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iIntegrator = testCase;
	m_iTestCase = m_iIntegrator;
	switch (m_iTestCase)
	{
	case 0:
		cout << "EULER !\n";
		break;
	case 1:
		cout << "LEAPFROG!\n";
		break;
	case 2:
		cout << "MIDPOINT !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_externalForce = inputWorld;	// not sure
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	m_iTestCase = m_iIntegrator;
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		eulerSimulation(timeStep);
		break;
	case 1:
		leapFrogSimulation(timeStep);
		break;
	case 2:
		midpointSimulation(timeStep);
		break;
	default:
		break;
	}
}



// Specific Functions
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	Point newPoint;
	newPoint.position = position;
	newPoint.velocity = velocity;
	newPoint.isFixed = isFixed;
	newPoint.acceleration = 0;	// just initialize, calculate its real value later
	points.push_back(newPoint);
	return points.size() - 1;	// return the index of new point
}

void MassSpringSystemSimulator::addSpring(int masspoint0, int masspoint1, float initialLength)
{
	Spring newSpring;
	newSpring.point0 = masspoint0;
	newSpring.point1 = masspoint1;
	newSpring.initialLength = initialLength;
	springs.push_back(newSpring);
}

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
	return points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return points[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}


// Simulation Functions
float MassSpringSystemSimulator::calculateCurrentLength(Vec3 point0, Vec3 point1)
{
	return sqrtf((point0.x - point1.x) * (point0.x - point1.x) + (point0.y - point1.y) * (point0.y - point1.y) + (point0.z - point1.z) * (point0.z - point1.z));

}

Vec3 MassSpringSystemSimulator::calculateDirection(Vec3 point0, Vec3 point1, float currentLength)
{
	return (point0 - point1) / currentLength;
}

void MassSpringSystemSimulator::calculateAcceleration(vector<Point>& points)
{
	vector<Vec3> accelerations;
	for (Spring s : springs) {
		int p0 = s.point0;
		int p1 = s.point1;

		float currentLength = calculateCurrentLength(points[p0].position, points[p1].position);
		Vec3 direction = calculateDirection(points[p0].position, points[p1].position, currentLength);
		Vec3 elasticForce = (-1) * m_fStiffness * (currentLength - s.initialLength) * direction;
		Vec3 force = elasticForce + m_externalForce;
		Vec3 acceleration = force / m_fMass;

		points[p0].acceleration = acceleration;
		points[p1].acceleration = (-1) * acceleration;
	}
}

void MassSpringSystemSimulator::eulerSimulation(float timestep)
{
	// calculate acceleration every step
	calculateAcceleration(points);

	for (int i = 0; i < points.size(); i++) {
		if (!points[i].isFixed) {
			points[i].position += timestep * points[i].velocity;
			points[i].velocity += timestep * points[i].acceleration;
		}
	}
}

void MassSpringSystemSimulator::leapFrogSimulation(float timestep)
{
	// TODO
}

void MassSpringSystemSimulator::midpointSimulation(float timestep)
{
	// calculate acceleration for t = 0
	calculateAcceleration(points);

	// t =0 -> t = h/2
	// use temp to record position and velocity to calculate acc(t=h/2)
	vector<Point> temp;
	for (int i = 0; i < points.size(); i++) {
		if (!points[i].isFixed) {
			Point p;
			p.position = points[i].position + 0.5 * timestep * points[i].velocity;
			p.velocity = points[i].velocity + 0.5 * timestep * points[i].acceleration;
			p.isFixed = false;
			p.acceleration = 0;	// just initialize, calculate its real value later
			temp.push_back(p);
		}
	}

	// calculate acceleration for t = h/2
	calculateAcceleration(temp);

	for (int i = 0; i < points.size(); i++) {
		if (!points[i].isFixed) {
			points[i].position += timestep * temp[i].velocity;
			points[i].velocity += timestep * temp[i].acceleration;
		}
	}
}



