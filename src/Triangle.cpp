//====================================
// brief: Triangle类，三角形的表示类
// author:sunxvming@163.com
// date:  2019-11-15
//====================================

#include "Triangle.h"
#include"Polygon.h"

Triangle::Triangle(int32_t p1, int32_t p2, int32_t p3):p1(p1),p2(p2),p3(p3),edges{ -1,-1,-1 }
{
}

void  circle_center(Point  *center, Point  pt[3], double  *radiu)
{
	double  x1, x2, x3, y1, y2, y3;
	double  x = 0;
	double  y = 0;

	x1 = pt[0].x;
	x2 = pt[1].x;
	x3 = pt[2].x;
	y1 = pt[0].y;
	y2 = pt[1].y;
	y3 = pt[2].y;

	x = ((y2 - y1)*(y3*y3 - y1*y1 + x3*x3 - x1*x1) - (y3 - y1)*(y2*y2 - y1*y1 + x2*x2 - x1*x1)) / (2 * (x3 - x1)*(y2 - y1) - 2 * ((x2 - x1)*(y3 - y1)));
	y = ((x2 - x1)*(x3*x3 - x1*x1 + y3*y3 - y1*y1) - (x3 - x1)*(x2*x2 - x1*x1 + y2*y2 - y1*y1)) / (2 * (y3 - y1)*(x2 - x1) - 2 * ((y2 - y1)*(x3 - x1)));

	center->x = x;
	center->y = y;
	*radiu = (pt[0].x - x)*(pt[0].x - x) + (pt[0].y - y)*(pt[0].y - y);
}


#define max( a, b ) ((a)>(b)?(a):(b))
#define min( a, b ) ((a)>(b)?(b):(a))
void Triangle::GenExtData(navmesh::Polygon* p)
{
	Point pt1 = p->GetPoint(p1);
	Point pt2 = p->GetPoint(p2);
	Point pt3 = p->GetPoint(p3);
	
	icenter.x = (pt1.x+pt2.x+pt3.x)/3;
	icenter.y = (pt1.y+pt2.y+pt3.y)/3;

	double maxx = max(pt1.x, pt2.x);
	double maxy =  max(pt1.y, pt2.y);
	double minx =  min(pt1.x, pt2.x);
	double miny =  min(pt1.y, pt2.y);

	if(pt3.x>maxx) maxx = pt3.x;
	if(pt3.y>maxy) maxy = pt3.y;
	if(pt3.x<minx) minx = pt3.x;
	if(pt3.y<miny) miny = pt3.y;

	lt.x = minx, lt.y = miny;
	rb.x = maxx, rb.y = maxy;
}

int32_t Triangle::Contain(navmesh::Polygon* p, Point pt)
{
	double x = pt.x, y = pt.y;
	if(x<lt.x) return 0;
	if(x>rb.x) return 0;
	if(y<lt.y) return 0;
	if(y>rb.y) return 0;

	Point pt1 = p->GetPoint(p1);
	Point pt2 = p->GetPoint(p2);
	Point pt3 = p->GetPoint(p3);

	Point v0 = pt3-pt1;
	Point v1 = pt2-pt1;
	Point v2 = pt-pt1;

	double dot00 = v0.Dot(v0) ;
    double dot01 = v0.Dot(v1) ;
    double dot02 = v0.Dot(v2) ;
    double dot11 = v1.Dot(v1) ;
    double dot12 = v1.Dot(v2) ; 
	double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;
    double u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return 0 ;
    }

    double v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return 0 ;
    }

    return u + v <= 1 ? 1:0 ;
}

Triangle::~Triangle()
{
}
