#ifndef VALUESTRUCTS_H
#define VALUESTRUCTS_H


enum ObjectType
{
	CUBE, SPHERE, // CYLINDER, STATIONARY,
};

struct Objects
{
	float posx, posy, posz;
	ObjectType type;
};

struct Size
{
	float sizex, sizey, sizez;
};

struct Wind
{
	float wx, wy, wz;

	//Wind(float x, float y, float z){wx = x; wy = y; wz = z;}
};

struct Gravity
{
	float gx, gy, gz;

	//Gravity(float x, float y, float z){gx = x; gy = y; gz = z;}
};

struct ForceOffset
{
	float offx, offy, offz;

	//ForceOffset(float x, float y, float z){offx = x; offy = y; offz = z;}
};

struct Fluid
{
	float c1, c2, density;
};

struct Material
{
	float restitution;
	float staticFriction;
	float dynamicFriction;

	Material(){};
	Material(float res, float stat, float dyn){ restitution = res; 
												staticFriction = stat;
												dynamicFriction = dyn;};
};

struct Plane
{
	float normalx, normaly, normalz;

	float p1[3], p2[3], p3[3];

	Material mat;

	Plane() {};
	Plane(float x, float y, float z, float p1in[3], float p2in[3], float p3in[3], float res, float stat, float dyn)
					{
						normalx = x;
						normaly = y;
						normalz = z;

						p1[0] = p1in[0];
						p1[1] = p1in[1];
						p1[2] = p1in[2];

						p2[0] = p2in[0];
						p2[1] = p2in[1];
						p2[2] = p2in[2];

						p3[0] = p3in[0];
						p3[1] = p3in[1];
						p3[2] = p3in[2];


						mat = Material(res, stat, dyn);
					}
};

struct Point
{
	float x, y, z;

	Point() {};
	Point(float xin, float yin, float zin) 
					{
						x = xin;
						y = yin;
						z = zin;
					};
}

#endif