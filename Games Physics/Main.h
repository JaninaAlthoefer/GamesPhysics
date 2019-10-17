#ifndef MAIN_H
#define MAIN_H

#include <windows.h>
#include <ctime>
#include <vector>
#include <fstream>
#include <iostream>
#include "../freeglut/include/GL/glut.h"

#include "Physics.h"



vector<string> splitString(string &str, char delimiter);
void readValues(const char *filename);
void buildWorld();

void checkStatus();
void writeToFile(ofstream *file, int i);

void endSimulation();

void processGround(vector<string> c);
void processGravity(vector<string> c);
void processFluid(vector<string> c);
void processSphere(vector<string> c);
void processCube(vector<string> c);
void processDimensions(vector<string> c);
void processMass(vector<string> c);
void processVelocity(vector<string> c);
void processRotationAxis(vector<string> c);
void processWind(vector<string> c);
//void processDrag(vector<string> c);
void processForceOffset(vector<string> c);
void processForce(vector<string> c);

void processPlane(vector<string> c);
void processPlaneMaterial(vector<string> c);
void processMaterial(vector<string> c);

static int win_width, win_height;
static float camDist = 100.0f, camHeight = 0.0f, camYaw = 0.0f;
vector<PhysicsObject> ph_objs;
int numObjs;
int elapsedTimeIn60thSec = 0;
bool stopPhysics = false, doingPhysics = false, init = true;

ofstream *objectfiles;

//Gravity g;
//Wind w;

vector<Gravity> gs;
vector<Wind> winds;
vector<Fluid> fluids;
vector<ForceOffset> fOffs;
vector<Objects> objs;
vector<Size> sizes;
vector<float> masses;
vector<Velocity> vels;
vector<RotationAxis> rotas;
vector<ForceDirection> forces;
vector<Plane> planes;
vector<Point> pPlanes;
vector<Material> pMats;
vector<Material> mats;


Gravity grav;
Wind win;
float ground = 0.0f;
//static float drag = 0.0f;
Fluid flui;


#endif