#include "RigidBodySystemSimulator.h"
#include "MassPoint.h"
#include "Spring.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_mouse = { 0,0 };
	m_trackmouse = { 0,0 };
	m_oldtrackmouse = { 0,0 };
}

/// *** UI functions *** ///

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4 (Euler),Demo 4 (Midpoint), Demo 5";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {


}

/*
Reacts to change in test case (e.g. the drop-down menu, or when "reset scene" is pressed).
- Sets up scene, and
- prints out a short message to standard output.
*/
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	// Pass argument to the attribute.
	m_iTestCase = testCase;

	// ** Setup Scene ** //
	switch (testCase)
	{
		// Setup for Demos 1-3 
	case 0: case 1: case 2:

		break;

		// Setup for Demo 3,4,5
	case 3: case 4: case 5:
	{
	
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

	case 5:
		cout << "Demo 5 selected.\n\n";
		break;
	default:
		break;
	}
}

/*
In Demo 4, allow the top left corner to be moved.
Based on the mouse movement during a click, add difference vector to the current position.
*/
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	if (m_iTestCase == 3 || m_iTestCase == 4 || m_iTestCase == 5)
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
			// Apply difference vector
			// Vec3 curpos = m_points[0]->getPosition();
			// m_points[0]->setPosition(curpos + inputWorld);
		}
	}

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 1:

		break;
	case 2:

		break;
	case 3:

		break;
	case 4:

		break;
	case 5:

		break;
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
