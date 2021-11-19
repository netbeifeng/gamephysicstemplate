#pragma once
#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include "Simulator.h"

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	int m_iIntegrator;



	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

#endif