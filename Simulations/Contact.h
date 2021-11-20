#ifndef CONTACT_h
#define CONTACT_h
#include "Simulator.h"

/* Created by Chang Luo 20.11.2021 */
class Contact {

public:
	Contact();
	Contact(Vec3 position, Vec3 normal, float depth, int body1, int body2);

	// Getters
	Vec3 getPosition();
	Vec3 getNormal();
	float getDepth();
	int getIdxOfBody1();
	int getIdxOfBody2();

	// Placeholder for Setters, if necessary

private:
	Vec3 m_position; // position vector
	Vec3 m_normal;   // normal vector
	float m_depth;   // penetration depth (invasion depth)
	int m_body1;     // index of BODY1
	int m_body2;     // index of BODY2
};

#endif