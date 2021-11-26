#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_externalForce = Vec3();
	m_mouse = {0,0};
	m_trackmouse = {0,0};
	m_oldtrackmouse = {0,0};
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3";
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
	for (RigidBody b : bodies)
	{
		DUC->drawRigidBody(b.getWorldMatrix());
	}

	switch (m_iTestCase)
	{
	case 0: case 1:
	{
		// Draw force vector
		Vec3 forcePoint = Vec3(0.3, 0.5, 0.25);
		Vec3 forceVector = Vec3(1, 1, 0);
		DUC->drawSphere(forcePoint, 0.05);
		DUC->beginLine();
		DUC->drawLine(forcePoint, Vec3(0,0,0), forcePoint+forceVector, Vec3(1,1,1));
		DUC->endLine();

		// Draw edges
		for (size_t i = 0; i < 8; i++)
		{
			DUC->drawSphere(bodies[0].getPointPosition(i), 0.01);
		}
		break;
	}
	default:
		break;
	}


}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	// * Setup Scene * //
	switch (testCase)
	{
	case 0: case 1:
	{
		bodies.clear();
		bodies.push_back(RigidBody(Vec3(1, 0.6, 0.5), 2));
		bodies[0].setOrientation(Quat(0, 0, M_PI / 2));
		// cout << "Set orientation to " << Quat(0, 0, M_PI/2) << "\n";

		bodies[0].addForce(Vec3(1, 1, 0), Vec3(0.3, 0.5, 0.25));
		// cout << "Force added.\n";
		// cout << "Torque: " << bodies[0].getTorque() << "\n";
		break;
	}
	case 2:
	{
		bodies.clear();

		bodies.push_back(RigidBody(Vec3(1, 1, 1), 1));
		bodies[0].setPosition(Vec3(1, 1, 1));
		bodies[0].addForce(Vec3(0,-2,0), Vec3(0.5, 1, 1));

		bodies.push_back(RigidBody(Vec3(5, 1, 5), 25));
		bodies[1].setPosition(Vec3(0, -1, 0));


		break;
	}
	default:
		break;
	}

	// * Output * //
	switch (testCase)
	{
	case 0:
	{
		cout << "Demo 1 selected.\n";
		int pt = 3;
		cout << "Original point position: " << bodies[0].getPointPosition(pt) << "\n";
		bodies[0].integrate(2);
		cout << "Integrated total point velocity: " << bodies[0].getPointVelocity(pt) << "\n";
		cout << "Body Angular Velocity: " << bodies[0].getAngularVelocity() << "\n";
		cout << "Body Linear Velocity: " << bodies[0].getLinearVelocity() << "\n";

		break;
	}
	case 1:
		cout << "Demo 2 selected.\n";
		break;
	case 2:
		cout << "Demo 3 selected.\n";
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 1)
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
			bodies[0].addForce(inputWorld, bodies[0].getPointPosition(0));
		}
	}

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:
		bodies[0].integrate(0.01);
		break;
	case 2:
	{
		// Integration
		for (size_t i=0; i<bodies.size(); i++)
		{
			bodies[i].integrate(timeStep);
		}

		// Collision detection
		Mat4 AM = bodies[0].getWorldMatrix();
		Mat4 BM = bodies[1].getWorldMatrix();
		CollisionInfo col = checkCollisionSAT(AM, BM);
		if (col.isValid)
		{
			// Resolve collision
			float c = 0.9;
			Vec3 vrel = bodies[1].getLinearVelocity() - bodies[0].getLinearVelocity();
			Vec3 n = col.normalWorld;
			float nume = -(1 + c) * dot(vrel, n);

			float Ma = bodies[0].getMass();
			float Mb = bodies[1].getMass();
			Mat4 Ia = bodies[0].getWorldInvInertia();
			Mat4 Ib = bodies[1].getWorldInvInertia();

			Vec3 xa = col.collisionPointWorld - bodies[0].getCenterPosition();
			Vec3 xb = col.collisionPointWorld - bodies[1].getCenterPosition();
			Vec3 isa = cross(Ia.transformVector(cross(xa, n)), xa);
			Vec3 isb = cross(Ib.transformVector(cross(xb, n)), xb);
			float denom = 1/Ma + 1/Mb + dot(isa + isb, n);

			auto J = nume / denom;

			bodies[0].applyImpulse(xa, J, -n);
			bodies[1].applyImpulse(xa, J, n);
		}
		break;
	}
	default:
		break;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;

}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies() { return 0; }
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return Vec3(); }
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { return Vec3(); }
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { return Vec3(); }
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {}
