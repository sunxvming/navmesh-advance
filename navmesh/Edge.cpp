//====================================
// brief: Edge类，表示经过剖分后的线段，一条线段可能包含一个或两个三角，包含两个点
// author:sunxvming@163.com
// date:  2019-11-22
//====================================

#include "Edge.h"
#include "Polygon.h"


Edge::Edge(int32_t t1, int32_t t2, int32_t p1, int32_t p2)
{
	triangles[0] = t1;
	triangles[1] = t2;
	points[0] = p1;
	points[1] = p2;
}

int32_t Edge::IsRestrain(Polygon* p)
{
	int32_t p1 = points[0];
	int32_t p2 = points[1];
	if (p->restrains.find((int32_t)PIndex(p1, p2)) == p->restrains.end()) {
		return 0;
	}
	else
	{
		return 1;
	}
}

Edge::~Edge()
{
}
