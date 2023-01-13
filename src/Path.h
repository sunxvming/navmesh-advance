//====================================
// brief: Path类，对外提供寻路的接口
// author:sunxvming@163.com
// date:  2019-11-22
//====================================
#ifndef NAVMESH_PATH_H
#define NAVMESH_PATH_H

#include<string>
#include<vector>
#include"Polygon.h"
#include"Math.h"

using namespace std;
class Polygon;
class Path
{
public:
	vector<Polygon> polygons;
	vector<double> points;
	vector<int > indexs;
	vector<double> finalpath;

	vector<double> cropoint;
	vector<int> restainline;
	vector<double> grideline;
	vector<int> triangleline;

private:
	//inline char* readFile(const char* file, int* size);
public:
	Path();
	//Path(const char* file);
	Path(char* cont, int size);
	const double* GetPoints(int* length);
	const int* GetIndexs(int* length);
	const int* GetTriangleline(int * lenght);
	vector<double> GetGrideLine(int* length);
	vector<Line> GetLines();

	const double* FindPaths(Point start, Point end, bool isturn, int* size);
	const double* FindCross(double startx, double starty, double facex, double facey);
	
	int GetNextEposId(int eposId, Polygon p);
	void Save(const char* filename);
	void Load(const char* filename);
	int CheckPath(double startx, double starty, double endx, double endy);
	~Path();
};

#endif   // NAVMESH_PATH_H