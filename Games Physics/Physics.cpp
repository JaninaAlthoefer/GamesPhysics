#include "Physics.h"
#include <cmath>

float Over6 = 1.f/6.f;

static float stepsize = 1.0f / 60.0f;

extern Gravity grav;
extern Wind win;
extern Fluid flui;

// this only works for spheres at the moment!
void Physics::timeStepProjectile(PhysicsObject *obj)
{
	//Init values
	float vel[3], pos[3], rot[3], aVel[3], sides[3];
	obj->getPosition(pos);
	obj->getVelocity(vel);
	obj->getRotationAxis(rot);
	obj->getSideLengths(sides);
	float r = sides[0];

	aVel[0] = vel[0] - win.wx;
	aVel[1] = vel[1] - win.wy;
	aVel[2] = vel[2] - win.wz;

	float aVelLen = sqrt((aVel[0]*aVel[0])+(aVel[1]*aVel[1])+(aVel[2]*aVel[2]));
	float rotLen = sqrt((rot[0]*rot[0])+(rot[1]*rot[1])+(rot[2]*rot[2]));

	float objDens = obj->getDensity();
	float f1 = (3.f*flui.c1*r)/(4.f*mathPi*r*r*r*objDens);
	float f2 = (3.f*flui.c2*r)/(4.f*mathPi*r*r*r*objDens);
	float f3 = (mathPi*0.5f*flui.density*r*r*r*rotLen*aVelLen);

	float rCrossVa[3];
	rCrossVa[0] = (rot[1]*aVel[2]) - (rot[2]*aVel[1]);
	rCrossVa[1] = (rot[2]*aVel[0]) - (rot[0]*aVel[2]);
	rCrossVa[2] = (rot[0]*aVel[1]) - (rot[1]*aVel[0]);

	float rCrossVaLen = sqrt((rCrossVa[0]*rCrossVa[0])+(rCrossVa[1]*rCrossVa[1])+(rCrossVa[2]*rCrossVa[2]));
	float rCrossVaLenInv;

	if (rCrossVaLen == 0.0f)
		rCrossVaLenInv = 0.0f;
	else
		rCrossVaLenInv= 1/rCrossVaLen;

	// RK4 step 1
	float k1p[3], k1v[3];
	k1p[0] = vel[0] * stepsize;
	k1p[1] = vel[1] * stepsize;
	k1p[2] = vel[2] * stepsize;

	k1v[0] = ((grav.gx) + (-f1 * aVel[0]) + (-f2 * aVelLen * aVel[0]) + (f3*rCrossVaLenInv*rCrossVa[0])) * stepsize;
	k1v[1] = ((grav.gy) + (-f1 * aVel[1]) + (-f2 * aVelLen * aVel[1]) + (f3*rCrossVaLenInv*rCrossVa[1])) * stepsize;
	k1v[2] = ((grav.gz) + (-f1 * aVel[2]) + (-f2 * aVelLen * aVel[2]) + (f3*rCrossVaLenInv*rCrossVa[2])) * stepsize;

	//RK4 step 2
	float aVelNew[3];
 	aVelNew[0] = aVel[0] + (0.5f * k1v[0]);
	aVelNew[1] = aVel[1] + (0.5f * k1v[1]);
	aVelNew[2] = aVel[2] + (0.5f * k1v[2]);
	float aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	float rCrossVaNew[3];
	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	float rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	float rCrossVaLenInvNew;

	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;


	float k2p[3], k2v[3]; 
	k2p[0] = (vel[0] * stepsize) + (k1v[0]*0.5f);
	k2p[1] = (vel[1] * stepsize) + (k1v[1]*0.5f);
	k2p[2] = (vel[2] * stepsize) + (k1v[2]*0.5f);

	k2v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k2v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k2v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;

	//RK4 step 3
	aVelNew[0] = aVel[0] + (0.5f * k2v[0]);
	aVelNew[1] = aVel[1] + (0.5f * k2v[1]);
	aVelNew[2] = aVel[2] + (0.5f * k2v[2]);
	aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	
	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;


	float k3p[3], k3v[3]; 
	k3p[0] = (vel[0] * stepsize) + (k2v[0]*0.5f);
	k3p[1] = (vel[1] * stepsize) + (k2v[1]*0.5f);
	k3p[2] = (vel[2] * stepsize) + (k2v[2]*0.5f);

	k3v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k3v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k3v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;

	//RK4 step 4
	aVelNew[0] = aVel[0] + k3v[0];
	aVelNew[1] = aVel[1] + k3v[1];
	aVelNew[2] = aVel[2] + k3v[2];
	aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	
	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;

	float k4p[3], k4v[3]; 
	k4p[0] = (vel[0] * stepsize) + (k3v[0]*0.5f);
	k4p[1] = (vel[1] * stepsize) + (k3v[1]*0.5f);
	k4p[2] = (vel[2] * stepsize) + (k3v[2]*0.5f);

	k4v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k4v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k4v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;


	//solution
	float kpNew[3], kvNew[3], posNew[3], velNew[3];
	
	kpNew[0] = Over6 * (k1p[0] + (2.f*k2p[0]) + (2.f*k3p[0]) + k4p[0]);
	kpNew[1] = Over6 * (k1p[1] + (2.f*k2p[1]) + (2.f*k3p[1]) + k4p[1]);
	kpNew[2] = Over6 * (k1p[2] + (2.f*k2p[2]) + (2.f*k3p[2]) + k4p[2]);

	kvNew[0] = Over6 * (k1v[0] + (2.f*k2v[0]) + (2.f*k3v[0]) + k4v[0]);
	kvNew[1] = Over6 * (k1v[1] + (2.f*k2v[1]) + (2.f*k3v[1]) + k4v[1]);
	kvNew[2] = Over6 * (k1v[2] + (2.f*k2v[2]) + (2.f*k3v[2]) + k4v[2]);

	posNew[0] = pos[0] + kpNew[0];
	posNew[1] = pos[1] + kpNew[1];
	posNew[2] = pos[2] + kpNew[2];

	velNew[0] = vel[0] + kvNew[0];
	velNew[1] = vel[1] + kvNew[1];
	velNew[2] = vel[2] + kvNew[2];

	obj->setVelocity(velNew);
	obj->setPosition(posNew);
}

void Physics::timeStepSliding(PhysicsObject *obj)
{
	/*/Init values
	float vel[3], pos[3], normal[3];
	obj->getPosition(pos);
	obj->getVelocity(vel);
	obj->getRotationAxis(rot);
	obj->getSideLengths(sides);
	float r = sides[0];

	aVel[0] = vel[0] - win.wx;
	aVel[1] = vel[1] - win.wy;
	aVel[2] = vel[2] - win.wz;

	float aVelLen = sqrt((aVel[0]*aVel[0])+(aVel[1]*aVel[1])+(aVel[2]*aVel[2]));
	float rotLen = sqrt((rot[0]*rot[0])+(rot[1]*rot[1])+(rot[2]*rot[2]));

	float objDens = obj->getDensity();
	float f1 = (3.f*flui.c1*r)/(4.f*mathPi*r*r*r*objDens);
	float f2 = (3.f*flui.c2*r)/(4.f*mathPi*r*r*r*objDens);
	float f3 = (mathPi*0.5f*flui.density*r*r*r*rotLen*aVelLen);

	float rCrossVa[3];
	rCrossVa[0] = (rot[1]*aVel[2]) - (rot[2]*aVel[1]);
	rCrossVa[1] = (rot[2]*aVel[0]) - (rot[0]*aVel[2]);
	rCrossVa[2] = (rot[0]*aVel[1]) - (rot[1]*aVel[0]);

	float rCrossVaLen = sqrt((rCrossVa[0]*rCrossVa[0])+(rCrossVa[1]*rCrossVa[1])+(rCrossVa[2]*rCrossVa[2]));
	float rCrossVaLenInv;

	if (rCrossVaLen == 0.0f)
		rCrossVaLenInv = 0.0f;
	else
		rCrossVaLenInv= 1/rCrossVaLen;

	// RK4 step 1
	float k1p[3], k1v[3];
	k1p[0] = vel[0] * stepsize;
	k1p[1] = vel[1] * stepsize;
	k1p[2] = vel[2] * stepsize;

	k1v[0] = ((grav.gx) + (-f1 * aVel[0]) + (-f2 * aVelLen * aVel[0]) + (f3*rCrossVaLenInv*rCrossVa[0])) * stepsize;
	k1v[1] = ((grav.gy) + (-f1 * aVel[1]) + (-f2 * aVelLen * aVel[1]) + (f3*rCrossVaLenInv*rCrossVa[1])) * stepsize;
	k1v[2] = ((grav.gz) + (-f1 * aVel[2]) + (-f2 * aVelLen * aVel[2]) + (f3*rCrossVaLenInv*rCrossVa[2])) * stepsize;

	//RK4 step 2
	float aVelNew[3];
 	aVelNew[0] = aVel[0] + (0.5f * k1v[0]);
	aVelNew[1] = aVel[1] + (0.5f * k1v[1]);
	aVelNew[2] = aVel[2] + (0.5f * k1v[2]);
	float aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	float rCrossVaNew[3];
	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	float rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	float rCrossVaLenInvNew;

	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;


	float k2p[3], k2v[3]; 
	k2p[0] = (vel[0] * stepsize) + (k1v[0]*0.5f);
	k2p[1] = (vel[1] * stepsize) + (k1v[1]*0.5f);
	k2p[2] = (vel[2] * stepsize) + (k1v[2]*0.5f);

	k2v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k2v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k2v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;

	//RK4 step 3
	aVelNew[0] = aVel[0] + (0.5f * k2v[0]);
	aVelNew[1] = aVel[1] + (0.5f * k2v[1]);
	aVelNew[2] = aVel[2] + (0.5f * k2v[2]);
	aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	
	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;


	float k3p[3], k3v[3]; 
	k3p[0] = (vel[0] * stepsize) + (k2v[0]*0.5f);
	k3p[1] = (vel[1] * stepsize) + (k2v[1]*0.5f);
	k3p[2] = (vel[2] * stepsize) + (k2v[2]*0.5f);

	k3v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k3v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k3v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;

	//RK4 step 4
	aVelNew[0] = aVel[0] + k3v[0];
	aVelNew[1] = aVel[1] + k3v[1];
	aVelNew[2] = aVel[2] + k3v[2];
	aVelNewLen = sqrt((aVelNew[0]*aVelNew[0])+(aVelNew[1]*aVelNew[1])+(aVelNew[2]*aVelNew[2]));

	rCrossVaNew[0] = (rot[1]*aVelNew[2]) - (rot[2]*aVelNew[1]);
	rCrossVaNew[1] = (rot[2]*aVelNew[0]) - (rot[0]*aVelNew[2]);
	rCrossVaNew[2] = (rot[0]*aVelNew[1]) - (rot[1]*aVelNew[0]);
	rCrossVaLenNew = sqrt((rCrossVaNew[0]*rCrossVaNew[0])+(rCrossVaNew[1]*rCrossVaNew[1])+(rCrossVaNew[2]*rCrossVaNew[2]));
	
	if (rCrossVaLenNew == 0.0f)
		rCrossVaLenInvNew = 0.0f;
	else
		rCrossVaLenInvNew= 1/rCrossVaLenNew;

	float k4p[3], k4v[3]; 
	k4p[0] = (vel[0] * stepsize) + (k3v[0]*0.5f);
	k4p[1] = (vel[1] * stepsize) + (k3v[1]*0.5f);
	k4p[2] = (vel[2] * stepsize) + (k3v[2]*0.5f);

	k4v[0] = ((grav.gx) + (-f1 * aVelNew[0]) + (-f2 * aVelNewLen * aVelNew[0]) + (f3*rCrossVaLenInvNew*rCrossVaNew[0])) * stepsize;
	k4v[1] = ((grav.gy) + (-f1 * aVelNew[1]) + (-f2 * aVelNewLen * aVelNew[1]) + (f3*rCrossVaLenInvNew*rCrossVaNew[1])) * stepsize;
	k4v[2] = ((grav.gz) + (-f1 * aVelNew[2]) + (-f2 * aVelNewLen * aVelNew[2]) + (f3*rCrossVaLenInvNew*rCrossVaNew[2])) * stepsize;


	//solution
	float kpNew[3], kvNew[3], posNew[3], velNew[3];
	
	kpNew[0] = Over6 * (k1p[0] + (2.f*k2p[0]) + (2.f*k3p[0]) + k4p[0]);
	kpNew[1] = Over6 * (k1p[1] + (2.f*k2p[1]) + (2.f*k3p[1]) + k4p[1]);
	kpNew[2] = Over6 * (k1p[2] + (2.f*k2p[2]) + (2.f*k3p[2]) + k4p[2]);

	kvNew[0] = Over6 * (k1v[0] + (2.f*k2v[0]) + (2.f*k3v[0]) + k4v[0]);
	kvNew[1] = Over6 * (k1v[1] + (2.f*k2v[1]) + (2.f*k3v[1]) + k4v[1]);
	kvNew[2] = Over6 * (k1v[2] + (2.f*k2v[2]) + (2.f*k3v[2]) + k4v[2]);

	posNew[0] = pos[0] + kpNew[0];
	posNew[1] = pos[1] + kpNew[1];
	posNew[2] = pos[2] + kpNew[2];

	velNew[0] = vel[0] + kvNew[0];
	velNew[1] = vel[1] + kvNew[1];
	velNew[2] = vel[2] + kvNew[2];

	obj->setVelocity(velNew);
	obj->setPosition(posNew);

	//*/
}

void Physics::changePlane(PhysicsObject *obj)
{

}