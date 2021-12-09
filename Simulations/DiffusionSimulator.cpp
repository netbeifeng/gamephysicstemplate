#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
}

const char * DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "Debug Mode", TW_TYPE_BOOLCPP, &m_debug, "");

	TwAddVarCB(DUC->g_pTweakBar, "Width", TW_TYPE_UINT32, [](const void* value, void* clientData) {
		DiffusionSimulator* simulator = static_cast<DiffusionSimulator*>(clientData);
		simulator->m_width = *static_cast<const unsigned int*>(value);
		simulator->notifyCaseChanged(simulator->m_iTestCase);
	}, [](void* value, void* clientData) {
		*static_cast<unsigned int*>(value) = static_cast<const DiffusionSimulator*>(clientData)->m_width;
	}, this, "step=1");

	TwAddVarCB(DUC->g_pTweakBar, "Height", TW_TYPE_UINT32, [](const void* value, void* clientData) {
		DiffusionSimulator* simulator = static_cast<DiffusionSimulator*>(clientData);
		simulator->m_height = *static_cast<const unsigned int*>(value);
		simulator->notifyCaseChanged(simulator->m_iTestCase);
	}, [](void* value, void* clientData) {
		*static_cast<unsigned int*>(value) = static_cast<const DiffusionSimulator*>(clientData)->m_height;
	}, this, "step=1");

	TwAddVarCB(DUC->g_pTweakBar, "Depth", TW_TYPE_UINT32, [](const void* value, void* clientData) {
		DiffusionSimulator* simulator = static_cast<DiffusionSimulator*>(clientData);
		simulator->m_depth = *static_cast<const unsigned int*>(value);
		simulator->notifyCaseChanged(simulator->m_iTestCase);
	}, [](void* value, void* clientData) {
		*static_cast<unsigned int*>(value) = static_cast<const DiffusionSimulator*>(clientData)->m_depth;
	}, this, "step=1");

	TwAddButton(DUC->g_pTweakBar, "Do One Step", [](void* clientData) { 
		DiffusionSimulator* simulator = static_cast<DiffusionSimulator*>(clientData);
		simulator->updateDemo1Grid(simulator->diffuseTemperatureExplicit(0.001f));
	}, this, "");

	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &m_alpha, "step=0.01   min=0.01");

}

void DiffusionSimulator::updateDemo1Grid(Grid* grid) {
	
	m_demo1_baseGrid = grid;
}

bool DiffusionSimulator::isBoundary(int xIdx, int yIdx, int zIdx, Vec3 size) {
	bool z0 = false;
	if (size.z >= 3) {
		z0 = zIdx == 0 && xIdx >= 0 && yIdx >= 0 || zIdx == m_depth - 1 && xIdx >= 0 && yIdx >= 0;
	}

	bool y0 = zIdx >= 0 && xIdx >= 0 && yIdx == 0 || yIdx == m_height - 1 && zIdx >= 0 && xIdx >= 0;
	bool x0 = zIdx >= 0 && xIdx == 0 && yIdx >= 0 || xIdx == m_width - 1 && yIdx >= 0 && zIdx >= 0;

	if (m_debug) {
		//std::cout << "Idx: " << xIdx << " " << yIdx << " " << zIdx << std::endl;
		//std::cout << "Bool: " << x0 << " " << y0 << " " << z0 << std::endl;
	}

	if (z0 || y0 || x0) {
		return true;
	}
	return false;
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	// to be implemented
	m_demo1_baseGrid = new Grid(m_width, m_height, m_depth);

	// initialize baseGrid
	for (int zIdx = 0; zIdx < m_depth; zIdx++) {
		for (int yIdx = 0; yIdx < m_height; yIdx++) {
			for (int xIdx = 0; xIdx < m_width; xIdx++) {
				if (!isBoundary(xIdx, yIdx, zIdx, m_demo1_baseGrid->getSize())) {
					m_demo1_baseGrid->setValue(xIdx, yIdx, zIdx, 1); // initialize grid with 1
				}
				else {
					m_demo1_baseGrid->setValue(xIdx, yIdx, zIdx, 0); // keep boundaries 0
				}
			}
		}
	}

	if (m_debug) {
		m_demo1_baseGrid->debugPrint();
	}

	//m_demo1_baseGrid = diffuseTemperatureExplicit(.01f);


	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* oldT = m_demo1_baseGrid; // 0 and 15 are boundaries

	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	Vec3 size = oldT->getSize();
	Grid* newT = new Grid(size.x, size.y, size.z); // 1 ~ 14 available

	//std::cout << size << std::endl;
	float deltaXSquared = 1 / pow(size.x, 2);
	float deltaYSquared = 1 / pow(size.y, 2);
	float deltaZSquared = 1 / pow(size.z, 2);

	if (size.z >= 3) {
		for (int zIdx = 1; zIdx < size.z - 1; zIdx++) {
			for (int yIdx = 1; yIdx < size.y - 1; yIdx++) {
				for (int xIdx = 1; xIdx < size.x - 1; xIdx++) {
					float oldVal = oldT->getValue(xIdx, yIdx, zIdx);
					float newVal = 0.f;
					float xComponent = oldT->getValue(xIdx + 1, yIdx, zIdx) - 2 * oldT->getValue(xIdx, yIdx, zIdx) + oldT->getValue(xIdx - 1, yIdx, zIdx);
					float yComponent = oldT->getValue(xIdx, yIdx + 1, zIdx) - 2 * oldT->getValue(xIdx, yIdx, zIdx) + oldT->getValue(xIdx, yIdx - 1, zIdx);
					float zComponent = oldT->getValue(xIdx, yIdx, zIdx + 1) - 2 * oldT->getValue(xIdx, yIdx, zIdx) + oldT->getValue(xIdx, yIdx, zIdx - 1);

					float cd = (xComponent / deltaXSquared) + (yComponent / deltaYSquared) + (zComponent / deltaZSquared);

					newVal = oldVal + timeStep * m_alpha * cd;

					if (newVal > m_demo1_baseGrid->getMaxTemperature()) {
						m_demo1_baseGrid->setMaxTemperature(newVal);
					}

					//std::cout << newVal << std::endl;
					newT->setValue(xIdx, yIdx, zIdx, newVal);
				}
			}
		}
	} else {
		for (int zIdx = 0; zIdx < size.z; zIdx++) {
			for (int yIdx = 1; yIdx < size.y - 1; yIdx++) {
				for (int xIdx = 1; xIdx < size.x - 1; xIdx++) {
					float oldVal = oldT->getValue(xIdx, yIdx, size.z - 1);
					float newVal = 0.f;
					float xComponent = oldT->getValue(xIdx + 1, yIdx, zIdx) - 2 * oldT->getValue(xIdx, yIdx, zIdx) + oldT->getValue(xIdx - 1, yIdx, zIdx);
					float yComponent = oldT->getValue(xIdx, yIdx + 1, zIdx) - 2 * oldT->getValue(xIdx, yIdx, zIdx) + oldT->getValue(xIdx, yIdx - 1, zIdx);

					float cd = (xComponent / deltaXSquared) + (yComponent / deltaYSquared) + 0;

					if (m_debug) {
						//std::cout << "XPART: " << (xComponent / deltaXSquared) << "   ";
						//std::cout << "YPART: " << (yComponent / deltaYSquared) << "   ";
						//std::cout << "ZPART: " << (0) << "   \n";

						//std::cout << "(" << xIdx << ", " << yIdx << ", " << 0 << ") " << oldVal << " + " << timeStep << " * " << m_alpha << " * " << cd << std::endl;
					}

					newVal = oldVal + timeStep * m_alpha * cd;

					if (newVal > m_demo1_baseGrid->getMaxTemperature()) {
						m_demo1_baseGrid->setMaxTemperature(newVal);
					}
					//std::cout << newVal << std::endl;
					newT->setValue(xIdx, yIdx, zIdx, newVal);
				}
			}
		}
	}


	if (m_debug) {
		std::cout << "Update explicit" << std::endl;
		newT->debugPrint();
	}
	//m_demo1_baseGrid = newT;
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
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
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		m_demo1_baseGrid = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects(Grid *grid) 
{
	// to be implemented
	//visualization
	float sphere_size = 2;
	Vec3 size = grid->getSize();
	for (int zIdx = 0; zIdx < size.z; zIdx++) {
		for (int yIdx = 0; yIdx < size.y; yIdx++) {
			for (int xIdx = 0; xIdx < size.x; xIdx++) {
				float x = (xIdx - (size.x - 1) * 0.5) * 0.5f;
				float y = yIdx * 0.5f + 1;
				float z = (zIdx - (size.z - 1) * 0.5) * 0.5f;

				float value = grid->getValue(xIdx, yIdx, zIdx);
				float maxTemperature = grid->getMaxTemperature();
				float scaledTemperature = value / maxTemperature;
				sphere_size = pow(0.35f * (1 + scaledTemperature), 4);
				//std::cout << value << " / " << maxTemperature << std::endl; // maybe take gradient here :@
				// cold -> blue RGB(0,0,1)
				// hot -> red RGB(1,0,0)
				Vec3 color = Vec3(sqrt(scaledTemperature), sqrt(1 - scaledTemperature), sqrt(1 - scaledTemperature));
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * color);
				DUC->drawSphere(Vec3(x, y, z), Vec3(sphere_size));
			}
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects(m_demo1_baseGrid);
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
