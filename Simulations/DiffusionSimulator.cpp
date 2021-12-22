#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(int n, int m, float alpha) {
	this->n = n;
	this->m = m;
	this->alpha = alpha;

	// initialize temperature to 0 with m rows and n cols
	vector<vector<float>> vec(n, vector<float>(m));
	temperature = vec;
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Explicit solver!\n";
		G = new Grid(16, 16, 0.5);  // real diffusion area is 1-14
		for (int i = 1; i <= 4; i++)  // set temperature for parts of the grid
		{
			for (int j = 1; j <= 4; j++)
			{
				G->temperature[i][j] = 100;
			}
		}
		break;
	}
	case 1:
	{
		cout << "Implicit solver!\n";
		G = new Grid(16, 16, 0.5);  // real diffusion area is 1-14
		for (int i = 1; i <= 4; i++)  // set temperature for parts of the grid
		{
			for (int j = 1; j <= 4; j++)
			{
				G->temperature[i][j] = 100;
			}
		}
		break;
	}
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep, int n, int m) {//add your own parameters
	Grid* newG = new Grid(n, m, 0.5);
	// to be implemented
	//make sure that the temperature in boundary cells stays zero

	vector<vector<float>> curT = G->temperature;
	vector<vector<float>> newT(G->n, vector<float>(G->m));
	for (int i = 1; i <= G->n - 2; i++) {
		for (int j = 1; j <= G->m - 2; j++) {
			newT[i][j] = curT[i][j] + G->alpha * timeStep *
				(curT[i + 1][j] - 2 * curT[i][j] + curT[i - 1][j]
					+ curT[i][j + 1] - 2 * curT[i][j] + curT[i][j - 1]);
		}
	}

	newG->temperature = newT;
	return newG;
}

int ConvertToOneD(int i, int j, int m) {
	return i * m + j;
}

void DiffusionSimulator::setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < G->n; i++) {
		for (int j = 0; j < G->m; j++) {
			int index = ConvertToOneD(i, j, G->m);
			b.at(index) = G->temperature[i][j];
		}
	}
}

void DiffusionSimulator::fillT(vector<Real> x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for (int i = 0; i < x.size(); i++) {
		int col = i / G->m;
		int row = i % G->m;
		G->temperature[col][row] = x[i];
	}
}

void setupA(SparseMatrix<Real>& A, double factor, int n, int m) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	for (int i = 0; i < n * m; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
	
	for (int i = 0; i < n * m; i++) {
		int col = i / m;
		int row = i % m;

		// only operate on 14x14 area
		if (col >= 1 && col <= (n - 2) && row >= 1 && row <= (m - 2)) {
			int index1 = ConvertToOneD(col + 1, row, m);
			int index2 = ConvertToOneD(col - 1, row, m);
			int index3 = ConvertToOneD(col, row + 1, m);
			int index4 = ConvertToOneD(col, row - 1, m);

			A.set_element(i, i, 1 + 4 * factor);
			A.set_element(i, index1, -factor);
			A.set_element(i, index2, -factor);
			A.set_element(i, index3, -factor);
			A.set_element(i, index4, -factor);
		}	
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = G->m * G->n ;  //N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	double factor = timeStep * G->alpha; // default delta x = 1
	setupA(*A, factor, G->n, G->m);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep, int m, int n)
{
	// to be implemented
	// update current setup for each frame

	// change m or n, reset a grid
	if (m != G->m || n != G->n) {
		G = new Grid(n, m, 0.5);
		for (int i = 1; i <= 4; i++)
		{
			for (int j = 1; j <= 4; j++)
			{
				G->temperature[i][j] = 100;
			}
		}
	}

	switch (m_iTestCase)
	{
	case 0:
		G = diffuseTemperatureExplicit(timeStep, n, m);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	for (int i = 1; i <= G->n - 2; i++) {
		for (int j = 1; j <= G->m - 2; j++) {
			// assume temperature is in [-5, 100]
			// map temperature to [0, 1]
			float diffuseColor = (G->temperature[i][j] - (-5)) / (100 - (-5));
			DUC->setUpLighting(Vec3(), Vec3(0, 0, 0), 100, Vec3(1, 1, 1) * diffuseColor);
			Vec3 pos = Vec3(0.1 * (i - (G->n - 1) / 2), 0.1 * (j - (G->m - 1) / 2), 0); // put it to the center
			DUC->drawSphere(pos, 0.1);
		}
	}
	
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
