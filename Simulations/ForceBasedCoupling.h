#ifndef FORCEBASEDCOUPLING_h
#define FORCEBASEDCOUPLING_h
#include "Simulator.h"
#include "RigidBody.h"
#include "MassPoint.h"
#include "Spring.h"


class ForceBasedCoupling :public Simulator {
public:
	// Construtors
	ForceBasedCoupling();

	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep, int m, int n);
	void simulateCollision();
	void onClick(int x, int y);
	void onMouse(int x, int y);

	int BPCollision(int b, int p);
	int BBCollision(int b1, int b2);

private:
	// Mass Spring
	vector<Point*> points;
	vector<Spring> springs;

	// Rigid Body
	vector<RigidBody> bodies;

	// UI
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// game
	int count;
	int total;
	int time;

};


#endif