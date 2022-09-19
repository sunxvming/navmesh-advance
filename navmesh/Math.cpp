//====================================
// brief: Math类，数学工具类
// author:sunxvming@163.com
// date:  2019-11-15
//====================================

#include "Math.h"

//求两条线段的交点
int sgn(double x)
{
	return x < -(1e-6) ? -1 : (x > 1e-6);
}

double Cross(Point p1, Point p2, Point p3, Point p4)
{
	return (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
}

double Area(Point p1, Point p2, Point p3)
{
	return Cross(p1, p2, p1, p3);
}

double fArea(Point p1, Point p2, Point p3)
{
	return fabs(Area(p1, p2, p3));
}

#define max( a, b ) ((a)>(b)?(a):(b))
#define min( a, b ) ((a)>(b)?(b):(a))
bool Math::Meet(Point p1, Point p2, Point p3, Point p4)
{
	return max(min(p1.x, p2.x), min(p3.x, p4.x)) <= min(max(p1.x, p2.x), max(p3.x, p4.x))
		&& max(min(p1.y, p2.y), min(p3.y, p4.y)) <= min(max(p1.y, p2.y), max(p3.y, p4.y))
		&& sgn(Cross(p3, p2, p3, p4) * Cross(p3, p4, p3, p1)) >= 0
		&& sgn(Cross(p1, p4, p1, p2) * Cross(p1, p2, p1, p3)) >= 0;
}


Point Math::Inter(Point p1, Point p2, Point p3, Point p4)
{
	//求定比分点,算出来有符号问题，行不通
	//double k = fArea(p1, p2, p3) / fArea(p1, p2, p4);
	//定比分公式
	//return Point((p3.x + k*p4.x) / (1 + k), (p3.y + k*p4.y) / (1 + k));

	//两点式方程求交点
	double a12 = p1.y - p2.y;
	double b12 = p2.x - p1.x;
	double c12 = p1.x * p2.y - p2.x * p1.y;

	double a34 = p3.y - p4.y;
	double b34 = p4.x - p3.x;
	double c34 = p3.x * p4.y - p4.x * p3.y;
	double D = a12 * b34 - a34 * b12;
	Point p;
	p.x = (b12 * c34 - b34 * c12) / D;
	p.y = (c12 * a34 - c34 * a12) / D;
	return p;

}


double Math::abs(double x)
{
	return x > 0 ? x : -x;
}