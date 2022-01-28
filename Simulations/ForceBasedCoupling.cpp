# include "ForceBasedCoupling.h"
# include "collisionDetect.h"

ForceBasedCoupling::ForceBasedCoupling()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();

	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };

	count = 0;
	total = 0;
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
		float r = 0.05;
		DUC->drawSphere(points[i]->getPosition(), Vec3(r, r, r));
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
		RigidBody body(0.8, Vec3(0.5, 0.5, 0.5), Vec3(0.5, 0.0, 0.0), 0.1);
		body.setOrientation(Quat(0, 0, M_PI / 2));
		body.setForce(Vec3(0, 10, 0), Vec3(1.5, 0.5, 1.5));
		bodies.push_back(body);

		// spring system
		points.clear();
		Point* pt0, * pt1;
		pt0 = new Point(Vec3(-0.2, 0.4, 0), Vec3(0.1, 0, 0), Vec3(0, 0, 0), 1, false, 0.8);
		pt1 = new Point(Vec3(-0.2, -0.5, 0), Vec3(-0.1, 0, 0), Vec3(0, 0, 0), 1, false, 0.8);
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
	case 1:
	{
		// rigid body
		bodies.clear();
		int k = 0;
		for (int i = -1; i <= 1; i++) {
			for (int j = 0; j <= 1; j++) {
				Vec3 x_cm = Vec3(i * 0.25 + 0.07, 0, j*0.4-0.2);
				RigidBody body(0.5, Vec3(0.1, 0.1, 0.1), x_cm, 0.5);
				body.setVelocity(Vec3(0, -0.2-k*0.1, 0));
				bodies.push_back(body);
				k++;
			}
		}
		//bodies[0].setOrientation(Quat(0, 0, M_PI / 2));
		total = bodies.size();
		
		// spring system
		points.clear();
		springs.clear();
		size_t p1 = -1;
		for (int i = -2; i <= 2; i++) {
			p1++;
			for (int j = -2; j <= 2; j++) {
				Point* p;
				Vec3 position = Vec3(0.3 * i, -0.4, 0.3 * j);
				p = new Point(position, Vec3(0, 0.5, 0), Vec3(0, 0, 0), 0.5, false, 0.8);
				points.push_back(p);
				
				if (j == 2) {
					continue;
				}
				Spring s(5, 0.3, 0, p1, p1+1);
				springs.push_back(s);
				p1++;				
			}
		}
		// four corners are fixed
		points[0]->setFixed();
		points[4]->setFixed();
		points[20]->setFixed();
		points[24]->setFixed();
		// complete springs to make a net
		for (int i = 0; i <= 4; i++) {
			for (int j = 1; j <= 4; j++) {
				size_t p1 = i + 5 * (j - 1);
				size_t p2 = i + 5 * j;
				Spring s(5, 0.3, 0, p1, p2);
				springs.push_back(s);
			}
		}	
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
	case 1:
	{
		// rigid body
		for (int i = 0; i < bodies.size(); i++) {
			bodies[i].integrate(timeStep);
			Vec3 x_cm = bodies[i].getXcm();
			// body constraint
			if (x_cm.y >= 0.4){
				bodies[i].clearVelocity();
				bodies[i].setVelocity(Vec3(0,-0.4,0));
			}
			else if (x_cm.x <= -0.5) {
				bodies[i].clearVelocity();
				bodies[i].setVelocity(Vec3(0.2, -0.4, 0));
			}
			else if (x_cm.x >= 0.5) {
				bodies[i].clearVelocity();
				bodies[i].setVelocity(Vec3(-0.2, -0.4, 0));
			}
			else if (x_cm.z <= -0.5) {
				bodies[i].clearVelocity();
				bodies[i].setVelocity(Vec3(0, -0.4, 0.2));
			}
			else if (x_cm.z >= 0.5) {
				bodies[i].clearVelocity();
				bodies[i].setVelocity(Vec3(0, -0.4, -0.2));
			}
		}

		// spring system
		for (Spring s : springs)
		{
			s.applyElasticForceToPoints(points);
		}
		for (Point* p : points)
		{
			p->integrate(timeStep);
			// point constraint
			Vec3 position = p->getPosition();
			if (position.y >= 0) {
				position.y = 0;
				p->setPosition(position);
				p->setVelocity(Vec3(0, -0.1, 0));
			}
			else if (position.y <= -0.6) {
				position.y = -0.6;
				p->setPosition(position);
				p->setVelocity(Vec3(0, 0.1, 0));
			}
			else if (position.x >= 0.5) {
				position.x = 0.5;
				p->setPosition(position);
				p->setVelocity(Vec3(-0.1, 0, 0));
			}
			else if (position.x <= -0.5) {
				position.x = -0.5;
				p->setPosition(position);
				p->setVelocity(Vec3(0.1, 0, 0));
			}
			else if (position.z >= 0.5) {
				position.z = 0.5;
				p->setPosition(position);
				p->setVelocity(Vec3(0, 0, -0.1));
			}
			else if (position.z <= -0.5) {
				position.z = -0.5;
				p->setPosition(position);
				p->setVelocity(Vec3(0, 0, 0.1));
			}

		}

		// collide
		simulateCollision();

		// game
		for (int i = 0; i < bodies.size(); i++) {
			Vec3 position = bodies[i].getXcm();
			if (position.y < -0.7) {
				count++;
				bodies.erase(bodies.begin() + i);	// delete this body
				if (count == total) {
					// goal: minimize the time
					cout << "Wow, you win in " << "!" << endl;
				}
			}
		}

		break;
	}
	default:
		break;
	}
}

void ForceBasedCoupling::externalForcesCalculations(float timeElapsed)
{
	switch (m_iTestCase)
	{
	case 0:
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
			float inputScale = 0.000005f;
			inputWorld = inputWorld * inputScale;

			//Vec3 force = inputWorld;
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
	case 1:
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

			int mid = points.size() / 2;
			Vec3 position = points[mid]->getPosition();
			points[mid]->setPosition(position + inputWorld);
		}
	}
	default:
		break;
	}
}



void ForceBasedCoupling::simulateCollision()
{
	switch (m_iTestCase)
	{
	case 0:
	{
		BPCollision(0, 0);
	}
	case 1:
	{		
		for (int i = 0; i < bodies.size(); i++) {
			// body and point collide
			for (int j = 0; j < points.size(); j++) {
				int result = BPCollision(i, j);
				if (result == 1) {
					break;
				}
			}

			// body and another body collide
			if (bodies.size() == 1) {
				continue;
			}
			for (int j = i + 1; j < bodies.size(); j++) {
				int result = BBCollision(i, j);
				if (result == 1) {
					break;
				}
			}
		}
	}
	default:
		break;
	}

}

int ForceBasedCoupling::BPCollision(int b, int p)
{
	CollisionInfo info = checkCollisionSAT(bodies[b].Obj2WorldMatrix(), points[p]->Obj2WorldMatrix());
	if (info.isValid) {
		cout << "BP collide!" << endl;
		Vec3 n = info.normalWorld;
		Vec3 x_collision = info.collisionPointWorld;
		float c_a = bodies[b].getC();
		float c_b = points[p]->getC();

		Vec3 v_a = bodies[b].getVcm() + cross(bodies[b].getW(), x_collision);
		Vec3 v_b = points[p]->getVelocity();
		Vec3 v_rel = v_a - v_b;
		float J1 = (-1) * (1 + c_a) * dot(v_rel, n);

		Vec3 x_a = x_collision - bodies[b].getXcm();
		Mat4 I_a = bodies[b].getI();
		float J2 = 1 / bodies[b].getMass() + 1 / points[p]->getMass();
		J2 += dot(cross(I_a * cross(x_a, n), x_a), n);
		float A = J1 / J2;

		bodies[b].setImpulse(A, x_a, n);
		points[p]->setVelocity(-0.1 * v_rel); // 0.1 is a scale		
		return 1;
	}
	return 0;
}

int ForceBasedCoupling::BBCollision(int b1, int b2)
{
	CollisionInfo info = checkCollisionSAT(bodies[b1].Obj2WorldMatrix(), bodies[b2].Obj2WorldMatrix());
	if (info.isValid) {
		cout << "BB collide!" << endl;
		Vec3 n = info.normalWorld;
		Vec3 x_collision = info.collisionPointWorld;
		float c = 0.0;		// 0 - 1  plastic - elastic

		Vec3 v_a = bodies[b1].getVcm() + cross(bodies[b1].getW(), x_collision);
		Vec3 v_b = bodies[b2].getVcm() + cross(bodies[b2].getW(), x_collision);
		Vec3 v_rel = v_a - v_b;
		float J1 = (-1) * (1 + c) * dot(v_rel, n);

		Vec3 x_a = x_collision - bodies[b1].getXcm();
		Vec3 x_b = x_collision - bodies[b2].getXcm();
		Mat4 I_a = bodies[b1].getI();
		Mat4 I_b = bodies[b2].getI();
		float J2 = 1 / bodies[b1].getMass() + 1 / bodies[b2].getMass();
		J2 += dot(cross(I_a * cross(x_a, n), x_a) + cross(I_b * cross(x_b, n), x_b), n);

		float J = J1 / J2;
		bodies[b1].setImpulse(J, x_a, n);
		bodies[b2].setImpulse(J, x_b, -n);
		return 1;
	}
	return 0;
}