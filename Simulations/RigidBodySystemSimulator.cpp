# include "RigidBodySystemSimulator.h"
# include "collisionDetect.h"

// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator() 
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

// Functions
const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < bodies.size(); i++) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(bodies[i].Obj2WorldMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Setup
	switch (testCase)
	{
	case 0: case 1:
	{
		bodies.clear();
		RigidBody body(2, Vec3(1, 0.6, 0.5), Vec3(0.0, 0.0, 0.0));
		body.setOrientation(Quat(0, 0, M_PI / 2));
		body.setForce(Vec3(1, 1, 0), Vec3(0.3, 0.5, 0.25));
		bodies.push_back(body);
		break;
	}
	case 2:
	{
		bodies.clear();
		/*
		RigidBody body1(0.2, Vec3(0.1, 0.1, 0.1), Vec3(-10.0, 0.0, 0.0));
		body1.setVelocity(Vec3(0.1, 0.0, 0.0));
		RigidBody body2(200000, Vec3(0.5, 0.5, 0.5), Vec3(0.0, 0.0, 0.0));
		body2.setVelocity(Vec3(0.0, 0.0, 0.0));
		*/
		RigidBody body1(2, Vec3(0.5, 0.5, 0.5), Vec3(-1.0, 0.0, 0.0));
		body1.setForce(Vec3(1, 1, 0), Vec3(-1.5,0.5,0.5));
		RigidBody body2(2, Vec3(0.5, 0.5, 0.5), Vec3(1.0, 0.0, 0.0));
		body2.setForce(Vec3(-1, 1, 0), Vec3(1.5,0.5,0.5));
		bodies.push_back(body1);
		bodies.push_back(body2);
		break;
	}
	}
	
	// Output
	switch (testCase)
	{
	case 0:
	// use braces to solve switch error: transfer of control bypasses initialization of
	{
		cout << "Demo 1 !" << "\n";
		bodies[0].integrate(2);
		//bodies[0].getWorldInfoOfPoint(Vec3(-0.3, -0.5, -0.25));
		break;
	}
	case 1:
	{
		cout << "Demo 2 !" << "\n";
		break;
	}
	case 2:
	{
		cout << "Demo 3 !" << "\n";
		break;
	}
	default:
	{
		cout << "Nothing !" << "\n";
		break;
	}
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 oldView = Vec3((float)m_oldtrackmouse.x, (float)-m_oldtrackmouse.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		Vec3 oldWorld = worldViewInv.transformVectorNormal(oldView);
		// find a proper scale!
		float inputScale = 0.000015f;
		inputWorld = inputWorld * inputScale;
		oldWorld = oldWorld * inputScale;
		// Apply difference vector
		if (m_iTestCase == 1) {
			Vec3 force = inputWorld;
			Vec3 position = oldWorld;
			bodies[0].setForce(force, position);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:
	{
		bodies[0].integrate(0.01);
		break;
	}
	case 2:
	{
		bodies[0].integrate(0.01);
		bodies[1].integrate(0.01);
		simulateCollision();
		break;
	}
	default:
		break;
	}
}

void RigidBodySystemSimulator::simulateCollision()
{
	CollisionInfo info = checkCollisionSAT(bodies[0].Obj2WorldMatrix(), bodies[1].Obj2WorldMatrix());
	if (info.isValid) {
		Vec3 n = info.normalWorld;
		Vec3 x_collision = info.collisionPointWorld;
		float c = 0.0;		// 0 - 1  plastic - elastic

		Vec3 v_a = bodies[0].getVcm() + cross(bodies[0].getW(), x_collision);
		Vec3 v_b = bodies[1].getVcm() + cross(bodies[1].getW(), x_collision);
		Vec3 v_rel = v_a - v_b;
		float J1 = (-1) * (1 + c) * dot(v_rel, n);

		Vec3 x_a = x_collision - bodies[0].getXcm();
		Vec3 x_b = x_collision - bodies[1].getXcm();
		Mat4 I_a = bodies[0].getI();
		Mat4 I_b = bodies[1].getI();
		float J2 = 1 / bodies[0].getMass() + 1 / bodies[1].getMass();
		J2 += dot(cross(I_a * cross(x_a, n), x_a) + cross(I_b * cross(x_b, n), x_b), n);
		
		float J = J1 / J2;
		bodies[0].setImpulse(J, x_a, n);
		bodies[1].setImpulse(-J, x_b, n);
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
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return bodies.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) 
{ 
	return bodies[i].getXcm(); 
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) 
{ 
	return bodies[i].getVcm();
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) 
{ 
	return bodies[i].getW();
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) 
{
	return bodies[i].setForce(force, loc);
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) 
{
	RigidBody body(mass, size, position);
	bodies.push_back(body);
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) 
{
	bodies[i].setOrientation(orientation);
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) 
{
	bodies[i].setVelocity(velocity);
}


