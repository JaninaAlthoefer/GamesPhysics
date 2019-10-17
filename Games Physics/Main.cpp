#include "Main.h"

void initRendering()
{
	glEnable(GL_DEPTH_TEST);

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	glClearColor(0.5f, 0.5f, 0.8f, 0.0f);

	readValues("InitialValues.txt");

	struct tm tTime;
	time_t tim;
	time(&tim);
	localtime_s(&tTime, &tim);

	numObjs = ph_objs.size();
	int nurbs = numObjs;

	objectfiles = new ofstream[numObjs];

	for (int i = 0; i < numObjs; i++)
	{
		string name = to_string(1900+tTime.tm_year)+"_"+to_string(1+tTime.tm_mon)+"_"+to_string(tTime.tm_mday)
						+"-"+to_string(tTime.tm_hour)+"-"+to_string(tTime.tm_min)+"-"+to_string(tTime.tm_sec)
						+"_"+"object"+to_string(i+1)+".txt";
		objectfiles[i].open(name, ios::app);
	}

	//print initial values
	float pos[3], sides[3], vels[3], rot[3], radius, mass, density;
	string type;

	for(int j = 0; j < numObjs; j++)
	{
		ph_objs[j].getPosition(pos);
		ph_objs[j].getSideLengths(sides);
		radius = sides[0];
		ph_objs[j].getVelocity(vels);
		ph_objs[j].getRotationAxis(rot);
		mass = ph_objs[j].getMass();
		density = ph_objs[j].getDensity();
		type = ph_objs[j].getTypeString();

		objectfiles[j] << "Initial Values" << endl;
		objectfiles[j] << "Type: " <<  type  << endl;
		objectfiles[j] << "Radius: " << radius << " Mass: " << mass << " Density: " << density << endl;
		objectfiles[j] << "Position XYZ: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
		objectfiles[j] << "Velocity XYZ: " << vels[0] << " " << vels[1] << " " << vels[2] << endl;
		objectfiles[j] << "Rotation XYZ: " << rot[0] << " " << rot[1] << " " << rot[2] << endl;
		objectfiles[j] << endl;
		objectfiles[j] << "Fluid: C1 " << flui.c1 << " C2 " << flui.c2 << " Density " << flui.density << endl;
		objectfiles[j] << "Wind XYZ: " << win.wx << " " << win.wy << " " << win.wz << endl;
		objectfiles[j] << "Gravity XYZ: " << grav.gx << " " << grav.gy << " " << grav.gz << endl;
		objectfiles[j] << "Ground: " << ground << endl;
		objectfiles[j] << endl;

		objectfiles[j] << "Start Simulation" << endl;

		objectfiles[j] << endl;
		}
	
 	cout << "Init Done"<<endl;

}

void drawScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(0.0f, -camHeight, -camDist);
	glRotatef(0.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(-camYaw, 0.0f, 1.0f, 0.0f);
	glTranslatef(0.0f, -15.0f, 0.0f);

	//do stuff here
	glColor3f( 1.0f, 0.0f, 0.0f );

	// start simulation
	if (doingPhysics)
	{
		elapsedTimeIn60thSec++;

		//if (elapsedTimeIn60thSec == 45)
			//doingPhysics = false;

		int done = 0;
		float pos[3], sides[3], radius;

		for(int j = 0; j < numObjs; j++)
		{
			ph_objs[j].getPosition(pos);
			ph_objs[j].getSideLengths(sides);
			radius = sides[0];

			if((pos[1]-radius)<=ground)
			{
				done++;
				continue;
			}

			Physics::timeStepSliding(&ph_objs[j]);

			writeToFile(&objectfiles[j], j);

		}

		if (done >= numObjs)
		{
			//stopPhysics = true;
			endSimulation();
		}

		cout << elapsedTimeIn60thSec << endl;
	}

	glutSwapBuffers();

	glutPostRedisplay();
}

void  keyboard(unsigned char key, int mx, int my)
{
	switch(key)
	{
	case 'w': {
					camHeight += 2.5f;
					if (camHeight > 30.0f) camHeight = 30.0f;
					break;
			  }
	case 's': {
					camHeight -= 2.5f;
					if (camHeight < -30.0f) camHeight = -30.0f;
					break;
			  }
	case 'a': {
					camYaw -= 2.5f;
					if (camYaw < 0.0f) camYaw += 360.0f;
					break;
			  }
	case 'd': {
					camYaw += 2.5f;
					if (camYaw > 360.0f) camYaw -= 360.0f;
					break;
			  }
	case 'q': {
					camDist -= 2.0f;
					if (camDist < 30.0f) camDist = 30.0f;
					break;
			  }
	case 'e': {
					camDist += 2.0f;
					if (camDist > 150.0f) camDist = 150.0f;
					break;
			  }
	case ' ': { 
					if (!stopPhysics)
					{
						doingPhysics = !doingPhysics; 
					}
					break;
			  }
	case 'i': {
					break;
			  }
	case 'o': {
					break;
			  }
	case 'l': { 
					break;
			}
	}
	
	glutPostRedisplay();
}

void  reshape(int w, int h)
{
    glViewport(0, 0, w, h);
	
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (double)w/h, 1, 200);

	win_width = w;
	win_height = h;
}

int  main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize( 900, 500 );
	glutInitWindowPosition( 1000, 1000 );
    glutCreateWindow("Games Physics");
	
	glutDisplayFunc(drawScene);
    glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	
	initRendering();

    glutMainLoop();
    return 0;
}


void readValues(const char *filename) 
{
	ifstream stream;
	stream.open(filename);
	
	if(!stream.is_open())
	{
		cout << "Couldn't open File" << endl;
		exit(1);
	}

    string line;

	while(getline(stream, line)) {
		vector<string> c = splitString(line,' ');

		if (c.size() == 0) continue;

		if(c[0].compare("ground") == 0) {
			processGround(c);
		} else if(c[0].compare("pPlane") == 0) {
			processPlane(c);
		} else if(c[0].compare("pMat") == 0) {
			processPlaneMaterial(c);
		} else if(c[0].compare("g") == 0) {
			processGravity(c);
		} else if(c[0].compare("fluid") == 0) {
			processFluid(c);
		} else if(c[0].compare("ballp") == 0) {
			processSphere(c);
		} else if(c[0].compare("cubep") == 0) {
			//processCube(c);
			;
		} else if(c[0].compare("size") == 0) {
			processDimensions(c);
		} else if(c[0].compare("mass") == 0) {
			processMass(c);
		} else if(c[0].compare("mat") == 0) {
			processMaterial(c);
		} else if(c[0].compare("vel") == 0) {
			processVelocity(c);
		} else if(c[0].compare("rota") == 0) {
			processRotationAxis(c);
		} else if(c[0].compare("wind") == 0) {
			processWind(c);
		} else if(c[0].compare("drag") == 0) {
			//processDrag(c);
			;
		} else if(c[0].compare("foff") == 0) {
			processForceOffset(c);
		} else if(c[0].compare("force") == 0) {
			processForce(c);
		}
		else {
			continue;
		}
	}
	
	stream.close();

	buildWorld();

	gs.clear();
	winds.clear();
	fOffs.clear();
	forces.clear();
	objs.clear();
	sizes.clear();
	masses.clear();
	pPlanes.clear();
	pMats.clear();
	mats.clear();
}

void buildWorld()
{
	//compute gravity
	if (gs.size() == 1)
	{
		grav = gs[0];
	}
	else
	{
		float x = 0.0f, y = 0.0f, z = 0.0f;

		for (int i = 0; i < gs.size(); i++)
		{
			x += gs[i].gx;
			y += gs[i].gy;
			z += gs[i].gz;

		}

		grav.gx = x;
		grav.gy = y;
		grav.gz = z;
	}

	//compute wind
	if (winds.size() == 1)
		win = winds[0];
	else
	{
		float x = 0.0f, y = 0.0f, z = 0.0f;

		for (int i = 0; i < gs.size(); i++)
		{
			x += winds[i].wx;
			y += winds[i].wy;
			z += winds[i].wz;

		}

		win.wx = x;
		win.wy = y;
		win.wz = z;
	}

	//compute fluid
	if (fluids.size() == 1)
		flui = fluids[0];
	else
	{
		float x = 0.0f, y = 0.0f, dens = 0.0f;

		for (int i = 0; i < fluids.size(); i++)
		{
			x += fluids[i].c1;
			y += fluids[i].c2;
			dens += fluids[i].density;
		}

		flui.c1 = x;
		flui.c2 = y;
		flui.density = dens;
	}

	//construct plane / slope
	int numPoints = pPlanes.size();

	if (numPoints >= 3)
	{
		for (int i = 2; i < pPlanes.size(); i++)
		{
			float vec1[3], vec2[3], vec3[3];

			vec1[0] = pPlanes[i-2].x - pPlanes[i-1].x;
			vec1[1] = pPlanes[i-2].y - pPlanes[i-1].y;
			vec1[2] = pPlanes[i-2].z - pPlanes[i-1].z;

			vec2[0] = pPlanes[i].x - pPlanes[i-1].x;
			vec2[1] = pPlanes[i].y - pPlanes[i-1].y;
			vec2[2] = pPlanes[i].z - pPlanes[i-1].z;

//			vec3[0] = pPlanes[i-2].x - pPlanes[i].x;
//			vec3[1] = pPlanes[i-2].y - pPlanes[i].y;
//			vec3[2] = pPlanes[i-2].z - pPlanes[i].z;

			float norm[3];

			norm[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
			norm[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
			norm[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];

			if (norm[1] < 0.0f)
			{
				norm[0] = - norm[0];
				norm[1] = - norm[1];
				norm[2] = - norm[2];
			}
		}

	}


	//build Objects
	int numObjs = objs.size();

	for (int i = 0; i < numObjs; i++)
	{
		SideLength sl;
		Velocity vel;
		RotationAxis rot;
		float mass;
		Material mat;

		PositionCoM p = PositionCoM(objs[i].posx, objs[i].posy, objs[i].posz);
		ObjectType ot = objs[i].type;

		if (sizes.empty() || sizes.size() < i)
			sl = SideLength(1.0f, 1.0f, 1.0f);
		else
			sl = SideLength(sizes[i].sizex, sizes[i].sizey, sizes[i].sizez);

		if (masses.empty() || masses.size() < i)
			mass = 1.0f;
		else
			mass = masses[i];

		if (vels.empty() || vels.size() < i)
			vel = Velocity(0.0f, 0.0f, 0.0f);
		else
		{


			vel = Velocity(vels[i].vel_x, vels[i].vel_y, vels[i].vel_z);
		}

		if (rotas.empty() || rotas.size() < i)
			rot = RotationAxis(0.0f, 0.0f, 0.0f);
		else
			rot = RotationAxis(rotas[i].rotA_x, rotas[i].rotA_y, rotas[i].rotA_z);

		if (mats.empty() || mats.size() < i)
			mat = Material(1.0f, 1.0f, 1.0f);
		else
			mat = Material(mats[i].restitution, mats[i].staticFriction, mats[i].dynamicFriction);

		ph_objs.push_back(PhysicsObject(p, sl, mass, vel, rot, mat, ot));

		/*
		if (fOffs.empty() || fOffs.size() < i)
		{
			float fpo[3] = {0.0f, 0.0f, 0.0f};
 			ph_objs[i].setForcePositionOffset(fpo);
		}
		else
		{
			float fpo[3] = {fOffs[i].offx, fOffs[i].offy, fOffs[i].offz};
			ph_objs[i].setForcePositionOffset(fpo);
		}

		if (forces.empty() || forces.size() < i)
		{
			float fd[3] = {0.0f, 0.0f, 0.0f};
 			ph_objs[i].setForceDirection(fd);
		}
		else
			ph_objs[i].setForceDirection(forces[i]);//*/
	}
}

void checkStatus()
{
	int done = 0;

	float pos[3], sides[3], radius;

	for (int i = 0; i < numObjs; i++)
	{
		ph_objs[i].getPosition(pos);
		ph_objs[i].getSideLengths(sides);
		radius = sides[0];

		if((pos[2]-radius)<=ground)
			done++;
	}

	if (done >= numObjs)
	{
		stopPhysics = true;
		endSimulation();
	}
}

void writeToFile(ofstream *file, int i)
{
	float pos[3], vels[3];

	ph_objs[i].getPosition(pos);
	ph_objs[i].getVelocity(vels);

	*file << "Time " << elapsedTimeIn60thSec << "/60 sec ";
	*file << " PosX " << pos[0] << " PosY " << pos[1] << " PosZ " << pos[2];
	*file << " VelX " << vels[0] << " VelY " << vels[1] << " VelZ " << vels[2] << endl;

}

void endSimulation()
{
	doingPhysics = false;
	stopPhysics = true;

	for (int i = 0; i < numObjs; i++)
	{
		objectfiles[i] << endl;
		objectfiles[i] << "Simulation End" << endl;
		objectfiles[i].close();
	}

	delete[] objectfiles;

	for (int i = numObjs -1 ; i < 0; i--)
	{
		delete &ph_objs[i];
	}

	ph_objs.clear();
}

vector<string> splitString(string &str, char delimiter)
{
	vector<string> words;
    string word;
    stringstream stream(str);

    while( getline(stream, word, delimiter) )
        words.push_back(word);

	return words;
}

void processGround(vector<string> c)
{
	if (ground == 0.0f)
		ground = atof(c[1].c_str());
}

void processGravity(vector<string> c)
{
	Gravity gg;

	gg.gx = atof(c[1].c_str());
	gg.gy = atof(c[2].c_str());
	gg.gz = atof(c[3].c_str());

	gs.push_back(gg);
}

void processFluid(vector<string> c)
{
	Fluid f;

	f.c1 = atof(c[1].c_str());
	f.c2 = atof(c[2].c_str());
	f.density = atof(c[3].c_str());

	fluids.push_back(f);
}

void processSphere(vector<string> c)
{
	Objects obj;

	obj.posx = atof(c[1].c_str());
	obj.posy = atof(c[2].c_str());
	obj.posz = atof(c[3].c_str());
	obj.type = ObjectType::SPHERE;

	objs.push_back(obj);
}

void processCube(vector<string> c)
{
	Objects obj;

	obj.posx = atof(c[1].c_str());
	obj.posy = atof(c[2].c_str());
	obj.posz = atof(c[3].c_str());
	obj.type = ObjectType::CUBE;

	objs.push_back(obj);
}

void processDimensions(vector<string> c)
{
	Size s;

	s.sizex = atof(c[1].c_str());
	s.sizey = atof(c[2].c_str());
	s.sizez = atof(c[3].c_str());

	sizes.push_back(s);
}

void processMass(vector<string> c)
{
	masses.push_back(atof(c[1].c_str()));
}

void processVelocity(vector<string> c)
{
	Velocity v;

	v.vel_x = atof(c[1].c_str());
	v.vel_y = atof(c[2].c_str());
	v.vel_z = atof(c[3].c_str());

	vels.push_back(v);
}

void processRotationAxis(vector<string> c)
{
	RotationAxis r;

	r.rotA_x = atof(c[1].c_str());
	r.rotA_y = atof(c[2].c_str());
	r.rotA_z = atof(c[3].c_str());

	rotas.push_back(r);
}

void processWind(vector<string> c)
{
	Wind ws;

	ws.wx = atof(c[1].c_str());
	ws.wy = atof(c[2].c_str());
	ws.wz = atof(c[3].c_str());

	winds.push_back(ws);
}

/*void processDrag(vector<string> c)
{
	if (drag == 0.0f)
		drag = atof(c[1].c_str());
}//*/

void processForceOffset(vector<string> c)
{
	ForceOffset f;

	f.offx = atof(c[1].c_str());
	f.offy = atof(c[2].c_str());
	f.offz = atof(c[3].c_str());

	fOffs.push_back(f);
}

void processForce(vector<string> c)
{
	float x, y, z;

	x = atof(c[1].c_str());
	y = atof(c[2].c_str());
	z = atof(c[3].c_str());

	forces.push_back(ForceDirection(x, y, z));
}

void processPlane(vector<string> c)
{
	float x, y, z;

	x = atof(c[1].c_str());
	y = atof(c[2].c_str());
	z = atof(c[3].c_str());

	pPlanes.push_back(Point(x, y, z));
}

void processPlaneMaterial(vector<string> c)
{
	float res, stat, dyn;

	res =  atof(c[1].c_str());
	stat =  atof(c[2].c_str());
	dyn =  atof(c[3].c_str());

	pMats.push_back(Material(res, stat, dyn));
}

void processMaterial(vector<string> c)
{
	float res, stat, dyn;

	res =  atof(c[1].c_str());
	stat =  atof(c[2].c_str());
	dyn =  atof(c[3].c_str());

	mats.push_back(Material(res, stat, dyn));
}