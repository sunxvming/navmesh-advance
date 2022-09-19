//====================================
// brief: Triangle类，三角形的表示类
// author:sunxvming@163.com
// date:  2019-11-15
//====================================

#ifndef NAVMESH_TRIANGLE_H
#define NAVMESH_TRIANGLE_H


#include "Point.h"
#include <stdint.h>

class Polygon;

class Triangle
{
public:
	int32_t p1;
	int32_t p2;
	int32_t p3;
	int32_t edges[3];
	Point icenter;//重心
	Point lt;
	Point rb;
public:
	Triangle(int32_t p1, int32_t p2, int32_t p3);
	Triangle() { 
		this->p1 = 0; 
		this->p2 = 0; 
		this->p3 = 0; 
		this->edges[0] = 0;
		this->edges[1] = 0;
		this->edges[2] = 0;
	}

	void GenExtData(Polygon* p);
	int32_t Contain(Polygon* p, Point pt);
	virtual ~Triangle();
};

#endif