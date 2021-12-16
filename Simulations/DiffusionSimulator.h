#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int n, int m, float alpha);


public:
	// Attributes
	// default delta x = delta y = 1
	int n;  // n cols
	int m;  // m rows
	float alpha;
	vector<vector<float>> temperature;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep, int m, int n);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timeStep, int n, int m);
	void diffuseTemperatureImplicit(float timeStep);

	void setupB(std::vector<Real>& b);
	void fillT(vector<Real> x);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	//save results of every time step
	// use *T because Grid *T = new Grid() in C++
	// it stores one grid, not all the grids in the simulation
	Grid *G; 
};

#endif