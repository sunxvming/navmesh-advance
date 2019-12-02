//====================================
// brief: Polygon类，代表寻路多边形的类，主要实现多边形的三角剖分和寻路
// author:sunxvming@163.com
// date:  2019-11-22
//====================================

#ifndef NAVMESH_POLYGON_H
#define NAVMESH_POLYGON_H

#include<vector>
#include"Point.h"
#include"Triangle.h"
#include"Edge.h"
#include"Circle.h"
#include"Math.h"
#include<unordered_map>
#include <stdint.h>


using namespace std;

#define  MAXPOINT 100000
#if _WINDOWS
	typedef unordered_map<int32_t, int32_t> Hash;
#else
	//typedef std::tr1::unordered_map<int32_t, int32_t> Hash;
	typedef unordered_map<int32_t, int32_t> Hash;
#endif
#ifndef _ASSERT
	#define _ASSERT(expr) ((void)0)
#endif

#define PIndex(p1, p2) (p1>p2?(p2*MAXPOINT+p1):(p1*MAXPOINT+p2))
#define SIndex(p1, p2, index) (p1=index/MAXPOINT, p2=index%MAXPOINT)
#define contain(p) ( lt == p || rt == p || rb == p || lb == p )

class Cell {
public:
	vector<int32_t> points;
	vector<int32_t> edges;
};

class Grid {
public:
	vector<Cell> cells;
	int32_t gride;
	double minx;
	double miny;
	double maxx;
	double maxy;
	int32_t xnum;
	int32_t ynum;
};

class Line {
public:
	Point p1;
	Point p2;
	float color[3];
};

class Polygon
{
public:
	vector<Point> points;//外围顶点,顺时针
	vector<Triangle> triangles;
	vector<Edge> edges;
	vector<int32_t> pointsnum;
	Grid grid;
	Hash restrains;
private:
	inline void Gen();
	inline void CreateTriangle(Hash* eindex, int32_t p1, int32_t p2, int32_t p3);
	inline int32_t CreateEdge(Hash* eindexs, int32_t triangle, int32_t p1, int32_t p2);
	inline int32_t FindDT(Grid* grid, int32_t p1, int32_t p2 );
	inline double Distance(Point from, int32_t edge, int32_t triangle, Point to);
	void Delaunay();
public:
	Point GetPoint(int32_t p);
	bool IsIntersect(int32_t edgepos, int32_t pa1, int32_t p1);
	bool JudgeIsVisible(int32_t pa1, int32_t p1);

	static Polygon CreateFromShort(char* cont, int32_t*len);
	//static Polygon CreateFromShort2(char* cont, int32_t*len);
	Polygon(double* pos, int32_t size);
	Polygon();		//--Polygon(char *pos)
	
	vector<Line> GetLines();
	vector<Line> GetGrideLines();
	vector<Point> GetCenters();

	int32_t IsFrist(int32_t p) { return p == 4; }
	int32_t FindTriangle(Point p);
	vector<Point> FindPath(Point from, Point to, bool isturn);	//true时为中点寻路，false为拐点寻路
	void GenExtData();

	void Save(FILE* file);
	virtual ~Polygon();
};

#endif // NAVMESH_POLYGON_H
