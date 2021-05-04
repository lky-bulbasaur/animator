#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>

typedef vector<std::pair<float, std::vector<Particle>*>>::iterator bakedStatesIterator;
typedef vector<Particle>::iterator bakedStateIterator;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
	bake_fps = 30;
	delta_t = 1 / bake_fps;
	bakedStates = new std::vector<std::pair<float, std::vector<Particle>*>>();
}





/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO
	clearBaked();
	delete bakedStates;
}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
    
	// TODO

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

	bake_start_time = t;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
    
	// TODO

	// These values are used by the UI
	simulate = false;
	dirty = true;

	bake_end_time = t;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
    
	// TODO

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{

	// TODO
	if (simulate) {
		bakeParticles(t);
	}
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
	for (bakedStatesIterator p = bakedStates->begin(); p != bakedStates->end(); ++p) {
		if (abs(p->first - t) <= TIME_EPSILON) {
			for (bakedStateIterator q = p->second->begin(); q != p->second->end(); ++q) {
				Vec3f coor = q->x;
				setDiffuseColor(1, 0, 0);
				glPointSize(rand() % 10);

				glPushMatrix();
					glBegin(GL_POINTS);
						glVertex3f(coor[0], coor[1], coor[2]);
					glEnd();
				glPopMatrix();
			}
		}
	}
}





/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{

	// TODO

	// Base case: There are no particles at all
	// No calculations are needed
	if (bakedStates->size() == 0) {
		bakedStates->push_back(std::pair<float, std::vector<Particle>*>(t, new std::vector<Particle>()));

		Particle particle;
		particle.x = indexTipCoor;
		particle.v = Vec3f(0, 0, 0);
		particle.f = Vec3f((rand() / (double)RAND_MAX - 0.5) * 2, -9.81, (rand() / (double)RAND_MAX - 0.5) * 2);
		particle.m = 1;
		(--bakedStates->end())->second->push_back(particle);

	// Normal case: Calculate new state of old particles then add a new particle
	} else {
		bakedStatesIterator prevBakedState = (--bakedStates->end());
		bakedStates->push_back(std::pair<float, std::vector<Particle>*>(t, new std::vector<Particle>()));
		(--bakedStates->end())->second->assign(prevBakedState->second->begin(), prevBakedState->second->end());
		for (bakedStateIterator p = (--bakedStates->end())->second->begin(); p != (--bakedStates->end())->second->end(); ++p) {
			float delta_t = t - prevBakedState->first;
			Vec3f xDeriv = p->v * delta_t;
			Vec3f vDeriv = p->f / p->m * delta_t;
			p->x += xDeriv;
			p->v += vDeriv;
		}

		Particle particle;
		particle.x = indexTipCoor;
		particle.v = Vec3f(0, 0, 0);
		particle.f = Vec3f((rand() / (double)RAND_MAX - 0.5) * 2, -9.81, (rand() / (double)RAND_MAX - 0.5) * 2);
		particle.m = 1;
		(--bakedStates->end())->second->push_back(particle);
	}
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	
	// TODO
	if (bakedStates != nullptr) {
		
		for (bakedStatesIterator p = bakedStates->begin(); p != bakedStates->end(); ++p) {
			if (p->second != nullptr) delete p->second;
		}
		delete bakedStates;
	}
	bakedStates = new std::vector<std::pair<float, std::vector<Particle>*>>();
	
}

void ParticleSystem::setIndexTipCoor(Vec3f c)
{
	indexTipCoor = c;
}





