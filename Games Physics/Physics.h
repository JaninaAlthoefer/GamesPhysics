#ifndef PHYSICS_H
#define PHYSICS_H

#include "PhysicsObjects.h"

//static float stepsize = 1.0f / 60.0f;

class Physics
{
private:


public: 

	static void timeStepProjectile(PhysicsObject *obj);
	//static void calculateCollisions(); need this later

	static void timeStepSliding(PhysicsObject *obj);
	static void changePlane(PhysicsObject *obj);

};

#endif