# include "ForceBasedCoupling.h"
# include "collisionDetect.h"

ForceBasedCoupling::ForceBasedCoupling()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

const char* ForceBasedCoupling::getTestCasesStr() {
	return "Demo 1, Demo 2";
}

void ForceBasedCoupling::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void ForceBasedCoupling::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void ForceBasedCoupling::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void ForceBasedCoupling::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void ForceBasedCoupling::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) 
{
	// draw rigid body
	for (int i = 0; i < bodies.size(); i++) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(bodies[i].Obj2WorldMatrix());
	}

	// draw spring system
	for (int i = 0; i < points.size(); i++) {
		DUC->drawSphere(points[i]->getPosition(), Vec3(0.05, 0.05, 0.05));  // r of sphere is 0.05
	}

	DUC->beginLine();
	for each (Spring s in springs)
	{
		DUC->drawLine(s.getP1(points)->getPosition(), Vec3(1, 1, 1), s.getP2(points)->getPosition(), Vec3(1, 1, 1));
	}
	DUC->endLine();
}

void ForceBasedCoupling::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	// Setup
	switch (testCase)
	{
	case 0:
	{
		// rigid body
		bodies.clear();
		RigidBody body(1, Vec3(0.5, 0.5, 0.5), Vec3(0.5, 0.0, 0.0), 0.1);
		//body.setOrientation(Quat(0, 0, M_PI / 2));
		//body.setForce(Vec3(0, 0, 100), Vec3(1.5, 0.5, 1.5));
		bodies.push_back(body);

		// spring system
		points.clear();
		Point* pt0, * pt1;
		pt0 = new Point(Vec3(-0.5, 0.4, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), 1, false, 0.8, 0.05);
		pt1 = new Point(Vec3(-0.5, -0.5, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), 1, false, 0.8, 0.05);
		pt0->addForce(Vec3(0, 10, 0),Vec3(-0.5+0.05, 0.4+0.05, -0.05) );
		points.push_back(pt0);
		points.push_back(pt1);
		for (Point* p : points)
		{
			p->addAcceleration(Vec3(0, -0.1, 0));
		}

		springs.clear();
		Spring s(4, 1, 0, 0, 1);
		springs.push_back(s);

		break;
	}
	default:
		break;
	}
}

void ForceBasedCoupling::simulateTimestep(float timeStep, int m, int n)
{
	switch (m_iTestCase)
	{
	case 0:
	{
		// rigid body
		for (int i = 0; i < bodies.size(); i++) {
			bodies[i].integrate(timeStep);
		}		

		// spring system
		for (Spring s : springs)
		{
			s.applyElasticForceToPoints(points);
		}
		for (Point* p : points)
		{
			p->integrate(timeStep);
		}

		// collide
		simulateCollision();
		break;
	}
	default:
		break;
	}
}

void ForceBasedCoupling::externalForcesCalculations(float timeElapsed)
{
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

		if (m_iTestCase == 0) {
			Vec3 force = inputWorld;
			//Vec3 p1 = points[0]->getPosition();
			//points[0]->setPosition(p1 + inputWorld);
			Vec3 p2 = bodies[0].getXcm();
			bodies[0].setXcm(p2 + inputWorld);

			/*
			// iterate to find which body/point the force apply on
			for (int i = 0; i < points.size(); i++) {
				if (points[i]->isInPoint(position) == 1) {
					Vec3 curPosition = points[i]->getPosition();
					points[i]->setPosition(curPosition + inputWorld);
					break;
				}
			}
			for (int i = 0; i < bodies.size(); i++) {
				if (bodies[i].isInBody(position) == 1) {
					Vec3 curPosition = bodies[i].getXcm();
					bodies[i].setXcm(curPosition + inputWorld);
					break;
				}
			}			
			*/
		}
	}
}

void ForceBasedCoupling::simulateCollision()
{



	CollisionInfo info = checkCollisionSAT(bodies[0].Obj2WorldMatrix(), points[0]->Obj2WorldMatrix());
	if (info.isValid) {
		cout << "collide!" << endl;
		Vec3 n = info.normalWorld;
		Vec3 x_collision = info.collisionPointWorld;		
		float c_a = bodies[0].getC();
		float c_b = points[0]->getC();

		Vec3 v_a = bodies[0].getVcm() + cross(bodies[0].getW(), x_collision);
		Vec3 v_b = points[0]->getVelocity() + cross(points[0]->getW(), x_collision);
		Vec3 v_rel = v_a - v_b;

		float A1 = (-1) * (1 + c_a) * dot(v_rel, n);
		float B1 = (-1) * (1 + c_b) * dot(-v_rel, -n); 

		Vec3 x_a = x_collision - bodies[0].getXcm();
		Vec3 x_b = x_collision - points[0]->getPosition();
		Mat4 I_a = bodies[0].getI();
		Mat4 I_b = points[0]->getI();

		float A2 = 1 / bodies[0].getMass() + 1 / points[0]->getMass();
		A2 += dot(cross(I_a * cross(x_a, n), x_a) + cross(I_b * cross(x_b, n), x_b), n);
		float A = A1 / A2;

		float B2 = 1 / bodies[0].getMass() + 1 / points[0]->getMass();
		B2 += dot(cross(I_a * cross(x_a, -n), x_a) + cross(I_b * cross(x_b, -n), x_b), -n);
		float B = B1 / B2;

		bodies[0].setImpulse(A, x_a, n);
		bodies[1].setImpulse(B, x_b, -n);
	}
}