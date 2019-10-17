#include "PhysicsObjects.h"

PhysicsObject::PhysicsObject(PositionCoM pos, SideLength len, float m, Velocity vel, RotationAxis rota, Material mat, ObjectType type)
{
	this->pos = pos;
	this->mass = m;
	this->velocity = vel;
	this->rotAxis = rota;
	this->material = mat;
	this->type = type;

	this->attachedTo = nullptr;

	/*if (type == ObjectType::CUBE)
	{
		this->sides = len;
		this->volume = this->sides.len_x*this->sides.len_y*this->sides.len_z;
	}
	else//*/ if (type == ObjectType::SPHERE)
	{
		float radius = oneOverThree * (len.len_x+len.len_y+len.len_z);
		this->sides = SideLength(radius, radius, radius);
		this->volume = (oneOverThree + 1.0f) * (radius*radius*radius) * mathPi;
	}
	/*else if (type == ObjectType::CYLINDER)
	{
		float radius = 0.5f * (len.len_y+len.len_z);
		this->sides = SideLength(len.len_x, radius, radius);
		this->volume = (radius*radius*this->sides.len_x) * mathPi;
	}
	else
	{
		this->mass = 0;
	}//*/

	
	this->density = (this->mass) / (this->volume);

	useC1 = true;
	useC2 = true;

	//remove this later
	this->collided = false;
}

PhysicsObject::~PhysicsObject()
{

}

void PhysicsObject::getPosition(float *positionXYZ)
{
	positionXYZ[0] = this->pos.pos_x;
	positionXYZ[1] = this->pos.pos_y;
	positionXYZ[2] = this->pos.pos_z;
}

void PhysicsObject::getVelocity(float *velocityXYZ)
{
	velocityXYZ[0] = this->velocity.vel_x;
	velocityXYZ[1] = this->velocity.vel_y;
	velocityXYZ[2] = this->velocity.vel_z;
}

void PhysicsObject::getSideLengths(float *lengthsXYZ)
{
	lengthsXYZ[0] = this->sides.len_x;
	lengthsXYZ[1] = this->sides.len_y;
	lengthsXYZ[2] = this->sides.len_z;
}

void PhysicsObject::getRotationAxis(float *rotAsXYZ)
{
	rotAsXYZ[0] = this->rotAxis.rotA_x;
	rotAsXYZ[1] = this->rotAxis.rotA_y;
	rotAsXYZ[2] = this->rotAxis.rotA_z;
}

void PhysicsObject::getMaterial(float *resStatDyn)
{
	resStatDyn[0] = this->material.restitution;
	resStatDyn[1] = this->material.staticFriction;
	resStatDyn[2] = this->material.dynamicFriction;
}

float PhysicsObject::getForce()
{
	return this->force;
}

void PhysicsObject::getForceDirection(float *forceDirXYZ)
{
	forceDirXYZ[0] = this->forceDirection.dir_x;
	forceDirXYZ[1] = this->forceDirection.dir_y;
	forceDirXYZ[2] = this->forceDirection.dir_z;
}

void PhysicsObject::getForcePositionOffset(float *forceOffXYZ)
{
	forceOffXYZ[0] = this->forceOffset.pos_x;
	forceOffXYZ[1] = this->forceOffset.pos_y;
	forceOffXYZ[2] = this->forceOffset.pos_z;
}

float PhysicsObject::getTorque()
{
	return this->torque;
}

void PhysicsObject::getTorqueDirection(float *torqueDirXYZ)
{
	torqueDirXYZ[0] = this->torqueDirection.dir_x;
	torqueDirXYZ[1] = this->torqueDirection.dir_y;
	torqueDirXYZ[2] = this->torqueDirection.dir_z;
}

void PhysicsObject::getAttached(PhysicsObject *att)
{
	att = attachedTo;
}

float PhysicsObject::getMass()
{
	return this->mass;
}

float PhysicsObject::getVolume()
{
	return this->volume;
}

float PhysicsObject::getDensity()
{
	return this->density;
}

bool PhysicsObject::getCollided()
{
	return this->collided;
}

ObjectType PhysicsObject::getType()
{
	return this->type;
}

string PhysicsObject::getTypeString()
{
	string string;

	if (this->type == ObjectType::SPHERE)
		string = "Sphere";
	else if (this->type == ObjectType::CUBE)
		string = "Cube";
	else
		string = "Unknown";

	return string;
}


void PhysicsObject::setPosition(float *positionXYZ)
{
	this->pos = PositionCoM(positionXYZ[0], positionXYZ[1], positionXYZ[2]);
}

void PhysicsObject::setVelocity(float *velocityXYZ)
{
	this->velocity = Velocity(velocityXYZ[0], velocityXYZ[1], velocityXYZ[2]);
}

void PhysicsObject::setSideLengths(float *lengthsXYZ)
{
	this->sides = SideLength(lengthsXYZ[0], lengthsXYZ[1], lengthsXYZ[2]);
}

void PhysicsObject::setRotationAxis(float *rotAsXYZ)
{
	this->rotAxis = RotationAxis(rotAsXYZ[0], rotAsXYZ[1], rotAsXYZ[2]);
}

void PhysicsObject::setMaterial(float *resStatDyn)
{
	this->material = Material(resStatDyn[0], resStatDyn[1], resStatDyn[2]);
}

void PhysicsObject::setForce(float f)
{
	this->force = f;
}

void PhysicsObject::setForceDirection(float *forceDirXYZ)
{
	this->forceDirection = ForceDirection(forceDirXYZ[0], forceDirXYZ[1], forceDirXYZ[2]);
}

void PhysicsObject::setForceDirection(ForceDirection fd)
{
	this->forceDirection = fd;
}

void PhysicsObject::setForcePositionOffset(float *forceOffXYZ)
{
	this->forceOffset = ForcePositionOffset(forceOffXYZ[0], forceOffXYZ[1], forceOffXYZ[2]);
}

void PhysicsObject::setTorque(float t)
{
	this->torque = t;
}

void PhysicsObject::setTorqueDirection(float *torqueDirXYZ)
{
	this->torqueDirection = TorqueDirection(torqueDirXYZ[0], torqueDirXYZ[1], torqueDirXYZ[2]);
}

void PhysicsObject::setAttached(PhysicsObject *att)
{
	attachedTo = att;
}

void PhysicsObject::setCollided(bool coll)
{
	this->collided = coll;
}

void PhysicsObject::setMass(float m)
{
	this->mass = m;
}

void PhysicsObject::setVolume(float v)
{
	this->volume = v;
}

void PhysicsObject::setDensity(float d)
{
	this->density = d;
}

void PhysicsObject::setType(ObjectType type)
{
	this->type = type;
}