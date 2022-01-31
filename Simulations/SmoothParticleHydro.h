#pragma once
#include "util/vectorbase.h"
#include<vector>
#include <functional>
#include <cmath>
using namespace GamePhysics;

class Particle
{
public:
	Particle(Vec3 pos, Vec3 vel, float m) {
		position = pos;
		force = Vec3(0, 0, 0);
		velocity = vel;
		mass = m;
		pressure_value = 1;
		density = 1;
	}
	Vec3 getPosition() { return position; }
	void integrate(float timestep)
	{
		position = position + timestep * velocity;

		Vec3 accel = force / mass;
		velocity = velocity + timestep * accel;

		force = Vec3(0, 0, 0);
	}
	void addForce(Vec3 f) { force += f; }
	void setDensity(float r) { density = r; }
	void setPressureValue(float p) { pressure_value = p; }
	float getMass() { return mass; }
	float getDensity() { return density; }
	float getPressureValue() { return pressure_value; }
	void addAcceleration(Vec3 a) {
		force += a * mass;
	}

	Vec3 getForce() { return force; }
	Vec3 getVelocity() { return velocity; }

private:
	Vec3 position;
	Vec3 force;
	Vec3 velocity;
	float mass;
	float density;
	float pressure_value;
};

/*
class AccelGrid
{
public:
	AccelGrid(Vec3 origin, Vec3 size, float cell_size)
	{
		// Initialize
		int x = (int)std::ceil(size.x / cell_size);
		int y = (int)std::ceil(size.y / cell_size);
		int z = (int)std::ceil(size.z / cell_size);
		arr = {};
		for (size_t i = 0; i < x; i++)
		{
			std::vector<std::vector<std::vector<Particle*>>> at_i = {};
			for (size_t j = 0; j < y; j++)
			{
				std::vector<std::vector<Particle*>> at_ij = {};
				for (size_t k = 0; k < z; k++)
				{
					std::vector<Particle*> at_ijk = {};
					at_ij.push_back(at_ijk);
				}
				at_i.push_back(at_ij);
			}
			arr.push_back(at_i);
		}
		this->cell_size = cell_size;
		grid_size = { x,y,z };
	}

	void iterate(std::function<void(Particle* p)> f, Vec3 pos)
	{
		Vec3 c = coordToCell(pos);

		for (Particle* p : arr.at(c.x).at(c.y).at(c.z))
			f(p);
		if (c.x > 0)
		{
			for (Particle* p : arr.at(c.x - 1).at(c.y).at(c.z))
				f(p);
			if (c.y > 0)
			{
				for (Particle* p : arr.at(c.x - 1).at(c.y-1).at(c.z))
					f(p);
				if (c.z > 0)
					for (Particle* p : arr.at(c.x-1).at(c.y-1).at(c.z-1))
						f(p);
				if (grid_size.z < c.z - 1)
					for (Particle* p : arr.at(c.x - 1).at(c.y - 1).at(c.z + 1))
						f(p);
			}
			if (grid_size.y < c.y - 1)
				for (Particle* p : arr.at(c.x - 1).at(c.y - 1).at(c.z - 1))
					f(p);
		}
	}

private:
	Vec3 coordToCell(Vec3 pos)
	{
		return { (int)std::floor(pos.x / cell_size),
			(int)std::floor(pos.y / cell_size),
			(int)std::floor(pos.z / cell_size) };
	}
	std::vector< std::vector< std::vector<std::vector<Particle*>>>> arr;
	float cell_size;
	Vec3 grid_size;
};
*/

class SmoothParticleHydro
{
public:
	SmoothParticleHydro(Vec3 position, Vec3 velocity, float radius) {
		Vec3 new_particle;
		kernel_halfrad = 0.04;
		for (float x = position.x - radius; x < position.x + radius; x += kernel_halfrad)
			for (float y = position.y - radius; y < position.y + radius; y += kernel_halfrad)
				for (float z = position.z - radius; z < position.z + radius; z += kernel_halfrad)
				{
					new_particle = Vec3(x, y, z);
					if (norm(position - new_particle) < radius)
						particles.push_back(new Particle(new_particle, velocity, 1));
				}
		rest_density = 50000;
		stiffness = 1;
		ds = 0;
	}
	void iterate(std::function<void(Vec3,float)> f)
	{
		for (Particle* p : particles)
			f(p->getPosition(), 2 * kernel_halfrad);
	}
	void integrate(float timeStep)
	{
		for (Particle* p : particles)
		{
			// Density
			float dens = 0;
			for (Particle* o : particles)
			{
				float q = norm(p->getPosition() - o->getPosition()) / kernel_halfrad;
				dens += W(q);
			}
			dens *= p->getMass(); // assuming all masses to b equal

			p->setDensity(dens);
			p->setPressureValue(stiffness * (std::pow(dens / rest_density, 7) - 1));
		}

		for (Particle* p : particles)
		{
			Vec3 pressure_force = Vec3(0, 0, 0);
			for (Particle* o : particles)
			{
				if (o == p) continue;
				Vec3 x_x = p->getPosition() - o->getPosition();
				float nxx = norm(x_x);
				float q = nxx / kernel_halfrad;
				Vec3 win = x_x / nxx;
				if (std::abs(o->getDensity()) > 0)
					pressure_force -= (p->getPressureValue() + o->getPressureValue()) / (2 * o->getDensity()) * win * nablaW(q);
			}
			// Ground
			//float floor_y = -0.5;
			//float q = (p->getPosition().y - floor_y) / kernel_halfrad;
			//pressure_force -= (p->getPosition())/(4) * Vec3(0, 1, 0) * nablaW(q);

			pressure_force *= p->getMass(); // assuming all masses to be equal


			p->addForce(pressure_force);
			p->addAcceleration(Vec3(0, -5, 0));
		}

		for (Particle* p : particles)
		{
			p->integrate(timeStep);
		}
	}

private:
	float W(float q) {
		if (2 <= q)
		{
			return 0.0;
		}
		else // q < 2
		{
			float f = 3 / (2 * std::pow(kernel_halfrad, 3) * M_PI);
			if (1 <= q)
			{
				return f * std::pow(2 - q, 3) / 6;
			}
			else if (0 <= q)
			{
				return f * (2 / 3 - std::pow(q, 2) +0.5*std::pow(q, 3));
			}
		}
	}
	float nablaW(float q) {
		if (2 <= q)
		{
			return 0.0;
		}
		else // q < 2
		{
			float f = 9 / (4 * std::pow(kernel_halfrad, 5) * M_PI);
			if (1 <= q)
			{
				return f * (- std::pow(2 - q, 2)) * kernel_halfrad / 3;
			}
			else if (0 <= q)
			{
				return f * (q - 4 / 3) * q * kernel_halfrad;
			}
		}
	}

	std::vector<Particle*> particles;
	float rest_density;
	float stiffness;
	float kernel_halfrad;

	int ds;
};

