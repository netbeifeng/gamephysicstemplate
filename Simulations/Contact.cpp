#include "Contact.h"

Contact::Contact() {}

Contact::Contact(Vec3 position, Vec3 normal, float depth, int body1, int body2) {
	m_position = position;
	m_normal = normal;
	m_depth = depth;
	m_body1 = body1;
	m_body2 = body2;
}

Vec3 Contact::getPosition() { return m_position; }
Vec3 Contact::getNormal() { return m_normal;  }
float Contact::getDepth() { return m_depth;  }
int Contact::getIdxOfBody1() { return m_body1;  }
int Contact::getIdxOfBody2() { return m_body2;  }
