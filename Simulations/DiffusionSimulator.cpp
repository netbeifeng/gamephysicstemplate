#include "DiffusionSimulator.h"
using namespace std;

Grid::Grid(int height, int width) {
	vector<float> row(width, 0.0f);
	vector<vector<float>> res(height, row);
	field = res;
}

void Grid::iterate(std::function<void(int, int, float)> f)
{
	for (int row = 0; row < field.size(); row++)
	{
		for (int col = 0; col < field[row].size(); col++)
		{
			f(row, col, field[row][col]);
		}
	}
}

void Grid::iterate(std::function<void(int, int, float, float, float, float, float)> f)
{
	for (int row = 0; row < field.size(); row++)
	{
		for (int col = 0; col < field[row].size(); col++)
		{
			float here = field[row][col];
			float left = col == 0 ? 0 : field[row][col - 1];
			float right = col == field[row].size() - 1 ? 0 : field[row][col + 1];
			float below = row == 0 ? 0 : field[row - 1][col];
			float above = row == field.size() - 1 ? 0 : field[row + 1][col];
			f(row, col, here, left, right, below, above);
		}
	}
}

void Grid::setTemp(int row, int col, float temp)
{
	field[row][col] = temp;
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
		cout << "Explicit solver!\n";
		T = new Grid();
		for (int row = 4; row < 14; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				T->setTemp(row, col, 50.0f);
			}
		}
		break;
	case 1:
		cout << "Implicit solver!\n";
		T = new Grid();
		for (int row = 4; row < 14; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				T->setTemp(row, col, 50.0f);
			}
		}
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Grid* newT = new Grid();
	
	auto pointwise = [newGrid = newT, d_t = timeStep](int row, int col, float here, float left, float right, float below, float above)
	{
		float t_a_ixiy22 = d_t/4;
		newGrid->setTemp(row, col, here + t_a_ixiy22*(left + right + below + above - 4*here));
	};
	T->iterate(pointwise);

	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return newT;
}

vector<Real> Grid::setupB(float timeStep, float factor) {
	vector<Real> b;

	this->iterate([&](int row, int col, float here, float left, float right, float below, float above) {
		b.push_back((here / timeStep + 0.5 * ((left + right - 2 * here) + (above + below - 2 * here)))/factor);
		});

	return b;
}

void Grid::fillT(vector<Real> x) {
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	int width = field[0].size();
	int row, col;
	for (int i = 0; i < x.size(); i++)
	{
		row = i / width;
		col = i % width;
		field[row][col] = x[i];
	}
}

SparseMatrix<Real> Grid::setupA(float timeStep, float factor) {
	size_t height = field.size();
	size_t width = field[0].size();
	SparseMatrix<Real> A(height * width);

	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < height * width; i++) {
		A.set_element(i, i, 1); // set diagonal
	}

	auto convert = [=](int row, int col) {return width * row + col; };

	int matrix_row;
	this->iterate([&](int row, int col, float _) {
		matrix_row = convert(row, col);
		if (col > 0) 		// Left
			A.set_element(matrix_row, convert(row, col - 1), -0.5f / factor);
		if (col < width - 1) // Right
			A.set_element(matrix_row, convert(row, col + 1), -0.5f / factor);
		if (row > 0)        // Above
			A.set_element(matrix_row, convert(row - 1, col), -0.5f / factor);
		if (row < height - 1) // Right
			A.set_element(matrix_row, convert(row + 1, col), -0.5f / factor);
		});

	return A;
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	float factor = 1 / timeStep + 2; // 1 / timeStep + 1 / (d_x * d_x) + 1 / (d_y * d_y);
	SparseMatrix<Real> A = T->setupA(timeStep, factor);
	std::vector<Real> b = T->setupB(timeStep, factor);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(b.size(), 0);

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	T->fillT(x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	auto draw = [duc = DUC](int row, int col, float temp)
	{
		float scale = 0.05;
		float grey = temp / 50;
		duc->setUpLighting(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.1, Vec3(grey, grey, grey));
		duc->drawSphere(Vec3(row*scale, col*scale, 0), scale);
	};

	T->iterate(draw);
	// to be implemented
	//visualization
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
