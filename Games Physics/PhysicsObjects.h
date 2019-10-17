#ifndef PHYSICSOBJECTS_H
#define PHYSICSOBJECTS_H


#include "ValueStructs.h"
#include <string.h>
#include <sstream>

using namespace std;

static float oneOverThree = 1.f/3.f;
static float mathPi = 3.141593f;

struct PositionCoM
{
	float pos_x, pos_y, pos_z;

	PositionCoM(){};
	PositionCoM(float x, float y, float z){pos_x = x; pos_y = y; pos_z = z;};
};

struct Velocity
{
	float vel_x, vel_y, vel_z;

	Velocity(){};
	Velocity(float x, float y, float z){vel_x = x; vel_y = y; vel_z = z;};
};

struct SideLength
{
	float len_x, len_y, len_z;

	SideLength(){};
	SideLength(float x, float y, float z){ len_x = x; len_y = y; len_z = z;};
};

struct RotationAxis
{
	float rotA_x, rotA_y, rotA_z;

	RotationAxis(){};
	RotationAxis(float x, float y, float z){ rotA_x = x; rotA_y = y; rotA_z = z;};
};

struct ForceDirection
{
	float dir_x, dir_y, dir_z;

	ForceDirection(){};
	ForceDirection(float x, float y, float z){dir_x = x; dir_y = y; dir_z = z;};
};

struct ForcePositionOffset
{
	float pos_x, pos_y, pos_z;

	ForcePositionOffset(){};
	ForcePositionOffset(float x, float y, float z){pos_x = x; pos_y = y; pos_z = z;};
};

struct TorqueDirection
{
	float dir_x, dir_y, dir_z;

	TorqueDirection(){};
	TorqueDirection(float x, float y, float z){dir_x = x; dir_y = y; dir_z = z;};
};

class PhysicsObject
{
private:
	PositionCoM pos;
	Velocity velocity;
	SideLength sides;
	RotationAxis rotAxis; // to be calculated from force & offset, later

	PhysicsObject *attachedTo;

	bool useC1, useC2; //does this make sense with floating points? figuring out if applicable is gonna be a mess

	float mass;
	float volume;
	float density; // calculated from size & mass
	float force;	// to be calculated from forceDirection
	ForceDirection forceDirection; 
	ForcePositionOffset forceOffset;
	float torque;	// to be calculated from torqueDirection
	TorqueDirection torqueDirection;

	Material material;
	bool collided;
	ObjectType type;
	
public:
	PhysicsObject(PositionCoM pos, SideLength len, float m, Velocity vel, RotationAxis rota, Material mat, ObjectType type);
	~PhysicsObject();

	void getPosition(float *positionXYZ);
	void getVelocity(float *velocityXYZ);
	void getSideLengths(float *lengthsXYZ);
	void getRotationAxis(float *rotAsXYZ);
	void getMaterial(float *resStatDyn);
	float getForce();
	void getForceDirection(float *forceDirXYZ);
	void getForcePositionOffset(float *forceOffXYZ);
	float getTorque();
	void getTorqueDirection(float *torqueDirXYZ);
	void getAttached(PhysicsObject *att);

	float getMass();
	float getVolume();
	float getDensity();
	bool getCollided();
	ObjectType getType();
	string getTypeString();
	
	void setPosition(float *positionXYZ);
	void setVelocity(float *velocityXYZ);
	void setSideLengths(float *lengthsXYZ);
	void setRotationAxis(float *rotAsXYZ);
	void setMaterial(float *resStatDyn);
	void setForce(float f);
	void setForceDirection(float *forceDirXYZ);
	void setForceDirection(ForceDirection fd);
	void setForcePositionOffset(float *forceOffXYZ);
	void setTorque(float t);
	void setTorqueDirection(float *torqueDirXYZ);
	void setAttached(PhysicsObject *att);

	void setCollided(bool coll);
	void setMass(float m);
	void setVolume(float v);
	void setDensity(float d);
	void setType(ObjectType type);
};

#endif