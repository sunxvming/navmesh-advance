//====================================
// brief: Math类，数学工具类
// author:sunxvming@163.com
// date:  2019-11-15
//====================================

#ifndef NAVMESH_MATH_H
#define NAVMESH_MATH_H

#include"Polygon.h"
class Math {
public:
	bool Meet(Point p1, Point p2, Point p3, Point p4);
	Point Inter(Point p1, Point p2, Point p3, Point p4);
	static double abs(double x);
};

#endif