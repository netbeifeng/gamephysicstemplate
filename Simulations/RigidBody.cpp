# include "RigidBody.h"


// Construtors
RigidBody::RigidBody(float b_mass, Vec3 b_size, Vec3 x_cm, float c)
{
	this->b_mass = b_mass;
	this->b_size = b_size;
	this->x_cm = x_cm;
	this->c = c;

	v_cm = Vec3(0, 0, 0);
	r = Quat(0, 0, 0, 0);
	L = Vec3(0, 0, 0);
	w = Vec3(0, 0, 0);
	q = Vec3(0, 0, 0);
	f = Vec3(0, 0, 0);

	float wi = b_size.x;	// width
	float he = b_size.y;	// height
	float de = b_size.z;	// depth
	// compute I0 inverse directly
	// note 0 can not be inversed
	I = Mat4(4 / (b_mass * (he * he + de * de)), 0, 0, 0,
			0, 4 / (b_mass * (wi * wi + de * de)), 0, 0,
			0, 0, 4 / (b_mass * (wi * wi + he * he)), 0,
			0, 0, 0, 0);
	
	/*
	// compute I0 according to 8 points
	float width = b_size.x;
	float height = b_size.y;
	float depth = b_size.z;

	vector<vector<double>> points;	// because x_cm is Vec3(double)
	vector<double> points_x = { x_cm.x - width / 2, x_cm.x - width / 2, x_cm.x + width / 2,x_cm.x + width / 2,x_cm.x - width / 2,x_cm.x - width / 2, x_cm.x + width / 2, x_cm.x + width / 2 };
	vector<double> points_y = { x_cm.y + height / 2, x_cm.y + height / 2, x_cm.y + height / 2, x_cm.y + height / 2, x_cm.y - height / 2 ,x_cm.y - height / 2, x_cm.y - height / 2, x_cm.y - height / 2 };
	vector<double> points_z = { x_cm.z + depth / 2, x_cm.z - depth / 2, x_cm.z - depth / 2, x_cm.z + depth / 2, x_cm.z + depth / 2, x_cm.z - depth / 2, x_cm.z - depth / 2, x_cm.z + depth / 2 };
	points.push_back(points_x);
	points.push_back(points_y);
	points.push_back(points_z);

	Mat4 C;	// initialize as 0
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			// i -> {x,y,z}		j -> {x,y,z}
			// compute dot product of two vectors
			// dot function is only for Vec3, so I write dotProduct function in vectorbase.h
			C.value[i][j] = (b_mass / 8) * dotProduct(points[i], points[j]);
		}
	}
	cout << "C is " << C << "\n";

	double trace = C.value[0][0] + C.value[1][1] + C.value[2][2];
	Mat4 Id;
	Id.initId();
	I = Id * trace - C;
	*/
}


// Functions
Mat4 RigidBody::Obj2WorldMatrix()
{
	Mat4 scaleMat, rotMat, translatMat, worldMatrix;
	scaleMat.initScaling(b_size.x, b_size.y, b_size.z);
	translatMat.initTranslation(x_cm.x, x_cm.y, x_cm.z);
	rotMat = r.getRotMat();
	worldMatrix = scaleMat * rotMat * translatMat;
	return worldMatrix;
}

void RigidBody::setOrientation(Quat r)
{
	this->r = this->r + r;
	
	Mat4 rotMat = this->r.getRotMat();
	Mat4 rotMatTranspose = this->r.getRotMat();
	rotMatTranspose.transpose();	// transpose function returns void
	I = rotMat * I * rotMatTranspose;
}

void RigidBody::setVelocity(Vec3 v_cm)
{
	this->v_cm = this->v_cm + v_cm;
}

void RigidBody::setForce(Vec3 force, Vec3 position)
{
	q = q + cross(position - x_cm, force);	// x_i = x_world - x_cm

	f = f + force;
}

void RigidBody::integrate(float h)
{
	x_cm = x_cm + h * v_cm;
	v_cm = v_cm + h * f / b_mass;
	r = r + (h / 2) * Quat(w.x, w.y, w.z, 0) * r;	// add 0 to do quaternion multiply
	L = L + h * q;
	w = I * L;		// I is the inverse

	//cout << "Linear velocity of the body is " << v_cm << "\n";
	//cout << "Angular velocity of the body is " << w << "\n";

	// clear force
	f = Vec3(0, 0, 0);
	q = Vec3(0, 0, 0);
}

void RigidBody::getWorldInfoOfPoint(Vec3 position)
{
	Mat4 rotMat = r.getRotMat();
	Vec3 x_i = x_cm + rotMat * position;
	Vec3 v_i = v_cm + cross(w, x_i);
	cout << "World position of the point is" << x_i << "\n";
	cout << "World linear velocity of the point is" << v_i << "\n";
}

Vec3 RigidBody::getVcm()
{
	return v_cm;
}

float RigidBody::getMass()
{
	return b_mass;
}

Mat4 RigidBody::getI()
{
	return I;
}

Vec3 RigidBody::getXcm()
{
	return x_cm;
}

void RigidBody::setImpulse(float J, Vec3 x_i, Vec3 n)
{
	v_cm = v_cm + J * n / b_mass;
	L = L + cross(x_cm, J * n);
}

Vec3 RigidBody::getW()
{
	return w;
}

/*
int RigidBody::isInBody(Vec3 p)
{
	int result = 0;

	// choose a proper r
	float r = 1;

	if (p.x >= x_cm.x - r && p.x <= x_cm.x + r
		&& p.y >= x_cm.y - r && p.y <= x_cm.y + r) {
		result = 1;
	}
	
	return result;
}
*/
void RigidBody::setXcm(Vec3 p)
{
	x_cm = p;
}

float RigidBody::getC()
{
	return c;
}

