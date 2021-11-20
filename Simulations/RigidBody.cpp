#include "RigidBody.h"

RigidBody::RigidBody() {}

RigidBody::RigidBody(Vec3 center_position, Vec3 size, float mass) {
	m_centerPosition = center_position;
	m_size = size;
	m_mass = mass;

	m_linearVelocity = Vec3();
	m_angularVelocity = Vec3();
	m_angularMomentum = Vec3();
	m_force = Vec3();
	m_forceLoc = Vec3();
	m_torque = Vec3();
	m_inertiaTensor = Mat4();

	preCompute();
}


Mat4 RigidBody::getToWorldMatrix() {
	//computeWorldMatrix();
	//std::cout << m_toWorld << std::endl;
	return m_toWorld;
}


Vec3 RigidBody::getAngularVelocity() {
	return m_angularVelocity;
}

Vec3 RigidBody::getLinearVelocity() {
	return m_linearVelocity;
}

Vec3 RigidBody::getCenterPosition() {
	return m_centerPosition;
}

void RigidBody::computeWorldMatrix() {
	Mat4 translation;
	Mat4 scale;
	Mat4 rotation;

	translation.initTranslation(m_centerPosition.x, m_centerPosition.y, m_centerPosition.z);
	scale.initScaling(m_size.x, m_size.y, m_size.z);
	rotation = m_rotation.getRotMat();
	//std::cout << "Rota " << m_rotation << std::endl;
	//std::cout << "Pos " << m_centerPosition << std::endl;

	m_toWorld = translation * scale * rotation;
}

void RigidBody::preCompute() {
	float x_cov = (m_mass / 8) * 2 * pow(m_size.x, 2);
	float y_cov = (m_mass / 8) * 2 * pow(m_size.y, 2);
	float z_cov = (m_mass / 8) * 2 * pow(m_size.z, 2);

	//std::cout << x_cov << std::endl;
	//std::cout << y_cov << std::endl;
	//std::cout << z_cov << std::endl;

	float trace = x_cov + y_cov + z_cov;
	
	//std::cout << trace << std::endl;

	Mat4 covarianceMatrix = Mat4();
	covarianceMatrix.initId();

	covarianceMatrix.value[0][0] = x_cov;
	covarianceMatrix.value[1][1] = y_cov;
	covarianceMatrix.value[2][2] = z_cov;
	
	//covarianceMatrix.value[0][0] = (1.0f / 12.0f) * (m_mass * (pow(m_size.y, 2) + pow(m_size.z, 2)));
	//covarianceMatrix.value[1][1] = (1.0f / 12.0f) * (m_mass * (pow(m_size.x, 2) + pow(m_size.z, 2)));
	//covarianceMatrix.value[2][2] = (1.0f / 12.0f) * (m_mass * (pow(m_size.x, 2) + pow(m_size.y, 2)));

	//trace = covarianceMatrix.value[0][0] + covarianceMatrix.value[1][1] + covarianceMatrix.value[2][2];

	Mat4 identityMatrix;
	identityMatrix.initId();

	Mat4 tmpInetriaTensor;
	tmpInetriaTensor = identityMatrix * trace - covarianceMatrix;
	tmpInetriaTensor.inverse();

	m_inertiaTensor = tmpInetriaTensor;
	//std::cout << (identityMatrix * trace - covarianceMatrix) << std::endl;
	//std::cout << m_inertiaTensor << std::endl;
}

void RigidBody::initialize() {
	//std::cout << "------ Initializing ------" << std::endl;
	Mat4 m_rotationTransposed = m_rotation.getRotMat();
	m_rotationTransposed.transpose();
	updateInetriaTensor(m_rotation.getRotMat() * m_inertiaTensor * m_rotationTransposed);

	updateAngularVelocity(m_inertiaTensor * m_angularMomentum);
	//std::cout << "------ End Of Init ------" << std::endl;
}

void RigidBody::setRotation(Quat rotation) {
	m_rotation = rotation;
	this->initialize();
}

void RigidBody::updateForce(Vec3 force) {
	m_force += force;
	//std::cout << "New Force = " << m_force << std::endl;
}

void RigidBody::updateForceLoc(Vec3 loc) {
	m_forceLoc = loc;
}

void RigidBody::updateTorque(Vec3 force, Vec3 loc) {
	m_torque += cross(force, loc);
	//std::cout << "New Torque = " << m_torque << std::endl;
}

void RigidBody::updateCenterPosition(Vec3 pos) {
	m_centerPosition += pos;
	//std::cout << "New CenterPos = " << m_centerPosition << std::endl;
}

void RigidBody::updateAngularVelocity(Vec3 av) {
	m_angularVelocity += av;
	//std::cout << "New AngularVel = " << m_angularVelocity << std::endl;
}

void RigidBody::updateLinearVelocity(Vec3 lv) {
	m_linearVelocity += lv;
	//std::cout << "New LinearVel = " << m_linearVelocity << std::endl;
}

void RigidBody::updateAngularMomentum(Vec3 am) {
	m_angularMomentum += am;
	//std::cout << "New AngularMom = " << m_angularMomentum << std::endl;
}

void RigidBody::updateInetriaTensor(Mat4 it) {
	m_inertiaTensor = it;
	//std::cout << "New InetriaTensor = " << m_inertiaTensor << std::endl;
}

void RigidBody::updateRotation(Quat r) {
	//std::cout << "\n " << r << "\n" << std::endl;
	m_rotation += r;
	//std::cout << "New Rotation = " << m_rotation << std::endl;
}

void RigidBody::integrate(float timeStep) {
	// Refer to lecture05-rigid-bodies-3D.pdf Slide 14
	//std::cout << "Integrating" << std::endl;

	updateTorque(m_force, m_forceLoc);

	updateCenterPosition(timeStep * m_linearVelocity); // update pos
	
	updateLinearVelocity(timeStep * (m_force / m_mass)); // update linear velocity

	// Integration Rotation
	Quat angularVelocityQuat = Quat(m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z, 0);
	updateRotation(0.5 * timeStep * angularVelocityQuat * m_rotation);

	// Integration Angular Momentum
	updateAngularMomentum(timeStep * m_torque);

	// Integration (Inverse) Inetria Tensor
	Mat4 m_rotationTransposed = m_rotation.getRotMat();
	m_rotationTransposed.transpose(); // return void -_-!! Why? NG!
	updateInetriaTensor(m_rotation.getRotMat() * m_inertiaTensor * m_rotationTransposed);

	// Integration Angualr Velocity
	updateAngularVelocity(m_inertiaTensor * m_angularMomentum);

	computeWorldMatrix();
	//std::cout << getToWorldMatrix() << std::endl;

	clear();
}

void RigidBody::clear() {
	m_force = Vec3();
	m_torque = Vec3();
}