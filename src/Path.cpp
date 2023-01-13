//====================================
// brief: Path类，对外提供寻路的接口
// author:sunxvming@163.com
// date:  2019-11-22
//====================================

#include "Path.h"

Path::Path()
{
}

//根据存入多边形数据的指针数据创建Path对象
Path::Path(char* cont, int size)
{
	int len = 0;
	while (size > 0)  //支持多个多边形
	{
		polygons.push_back(navmesh::Polygon::CreateFromShort(cont, &len));
		cont += len;
		size -= len;
	}
}

//把经过处理的地图多边形信息以二进制形式保存到文件中
void Path::Save(const char* filename)
{
	FILE* file;
	errno_t err;
	err = fopen_s(&file, filename, "wb");
	if (err != 0) {
		return;
	}
	size_t len = polygons.size();
	fwrite(&len, sizeof(size_t), 1, file);
	for (int i = 0; i < len; i++)
	{
		polygons[i].Save(file);
	}
	fclose(file);
}

//从文件中读取信息
void Path::Load(const char* filename) {
	FILE* file;
	errno_t err;
	err = fopen_s(&file, filename, "rb");
	if (err != 0) {
		return;
	}
	size_t len;
	fread(&len, 1, sizeof(size_t), file);
	for (int i = 0; i < len; i++) {
		navmesh::Polygon polygon;
		int pointsLen;
		fread(&pointsLen, 1, sizeof(int), file);
		for (int j = 0; j < pointsLen; j++) {
			Point point;
			fread(&(point), 1, sizeof(Point), file);
			polygon.points.push_back(point);
		}

		int triangeLen;
		fread(&triangeLen, 1, sizeof(int), file);
		for (int j = 0; j < triangeLen; j++) {
			Triangle tri;
			fread(&(tri), 1, sizeof(Triangle), file);
			polygon.triangles.push_back(tri);
		}

		int edgeLen;
		fread(&edgeLen, 1, sizeof(int), file);
		for (int j = 0; j < edgeLen; j++) {
			Edge edge;
			fread(&(edge), 1, sizeof(Edge), file);
			polygon.edges.push_back(edge);
		}

		int pLen;
		fread(&pLen, 1, sizeof(int), file);
		for (int j = 0; j < pLen; j++) {
			int num;
			fread(&(num), 1, sizeof(int), file);
			polygon.pointsnum.push_back(num);
		}

		int cellsize;
		fread(&cellsize, 1, sizeof(int), file);
		Grid grid;
		for (int j = 0; j < cellsize; j++)
		{
			Cell cel;
			int gridpoLen;
			fread(&gridpoLen, 1, sizeof(int), file);
			for (int k = 0; k < gridpoLen; k++) {
				int pk;
				fread(&(pk), 1, sizeof(int), file);
				cel.points.push_back(pk);
			}

			int gridedLen;
			fread(&gridedLen, 1, sizeof(int), file);
			for (int k = 0; k < gridedLen; k++) {
				int ek;
				fread(&(ek), 1, sizeof(int), file);
				cel.edges.push_back(ek);
			}
			grid.cells.push_back(cel);
		}
		int gride;
		fread(&(gride), 1, sizeof(int), file);
		grid.gride = gride;

		double minx, maxx, miny, maxy;
		fread(&(minx), 1, sizeof(double), file);
		grid.minx = minx;
		fread(&(maxx), 1, sizeof(double), file);
		grid.maxx = maxx;
		fread(&(miny), 1, sizeof(double), file);
		grid.miny = miny;
		fread(&(maxy), 1, sizeof(double), file);
		grid.maxy = maxy;

		int xnum, ynum;
		fread(&(xnum), 1, sizeof(int), file);
		grid.xnum = xnum;
		fread(&(ynum), 1, sizeof(int), file);
		grid.ynum = ynum;

		polygon.grid = grid;
		polygons.push_back(polygon);
	}
	fclose(file);
}

//得到所有点的数据
const double* Path::GetPoints(int* length) {
	points.clear();

	for (int i = 0; i < polygons.size(); i++) {
		for (int j = 0; j < polygons[i].points.size(); j++) {
			Point p = polygons[i].points[j];
			points.push_back(p.x);
			points.push_back(p.y);
		}
	}
	*length = points.size();
	return points.data();
}

//得到所有经过三角剖分的边的数据
const int* Path::GetIndexs(int* length) {
	indexs.clear();
	int vector = 0;
	for (int i = 0; i < polygons.size(); i++) {
		for (int j = 0; j < polygons[i].triangles.size(); j++)
		{
			indexs.push_back(polygons[i].triangles[j].p1 + vector);
			indexs.push_back(polygons[i].triangles[j].p2 + vector);
			indexs.push_back(polygons[i].triangles[j].p3 + vector);
		}
		vector += polygons[i].points.size();
	}
	*length = indexs.size();
	return indexs.data();
}

// 寻路
// param：
// isturn：true时为中点寻路，false为拐点寻路
// size：出参，寻路经过的点的数量
// return：
// 寻路经过的点的位置坐标
const double* Path::FindPaths(Point start, Point end, bool isturn, int* size) {
	finalpath.clear();
	int startPIndex = -1;
	int endPIndex = -1;
	for (int i = 0; i < polygons.size(); i++) {

		double minx = polygons[i].grid.minx;
		double maxx = polygons[i].grid.maxx;
		double miny = polygons[i].grid.miny;
		double maxy = polygons[i].grid.maxy;
		if ((start.x >= minx &&start.x <= maxx) && (start.y >= miny &&start.y <= maxy)) {
			startPIndex = i;
		}
		if ((end.x >= minx &&end.x <= maxx) && (end.y >= miny &&end.y <= maxy)) {
			endPIndex = i;
		}
	}
	if ((startPIndex != endPIndex) || (startPIndex < 0) || (endPIndex < 0)) {
		return 0;
	}
	else {
		int pIndex = startPIndex;

		int startTIndex = polygons[pIndex].FindTriangle(start);
		if (startTIndex < 0) return 0;

		int endTIndex = polygons[pIndex].FindTriangle(end);
		if (endTIndex < 0) return 0;

		vector<Point> f = polygons[pIndex].FindPath(start, end, isturn);

		for (int i = 0; i < f.size(); i++) {
			finalpath.push_back(f[i].x);
			finalpath.push_back(f[i].y);
		}
		*size = f.size() * 2;
		return finalpath.data();
	}
	return 0;
}


//得到edge的下一个点
int Path::GetNextEposId(int eposId, navmesh::Polygon p) {
	int sum = 0;
	int nextId = 0;
	for (int i = 0; i < p.pointsnum.size(); i++)
	{
		sum += p.pointsnum[i];
		if (eposId == sum - 1) {
			nextId = eposId - (p.pointsnum[i] - 1);
			break;
		}
		else if (eposId < sum - 1) {
			nextId = eposId + 1;
			break;
		}
		else if (eposId > sum - 1) {
			continue;
		}
	}
	return nextId;
}

//通过start起点和face朝向点找到与边界相交的点
const double* Path::FindCross(double startx, double starty, double facex, double facey) {
	cropoint.clear();
	//如果facex与facey同为0，则忽略，直接返回start点
	if (facex == 0 && facey == 0) {
		cropoint.push_back(startx);
		cropoint.push_back(starty);
	}

	Point start = Point(startx, starty);
	Point face = Point(startx + facex, starty + facey);
	int sIndex = -1;
	int stIndex = -1;
	for (int i = 0; i < polygons.size(); i++) {
		double minx = polygons[i].grid.minx;
		double maxx = polygons[i].grid.maxx;
		double miny = polygons[i].grid.miny;
		double maxy = polygons[i].grid.maxy;
		if ((startx >= minx &&startx <= maxx) && (starty >= miny &&starty <= maxy)) {
			sIndex = i;
			stIndex = polygons[sIndex].FindTriangle(start);
		}
	}
	if (stIndex < 0) return 0;

	//Point vector = Point(facex - startx, facey - starty);		//向量
	double minx, maxx, miny, maxy;
	navmesh::Polygon p = polygons[sIndex];
	minx = p.grid.minx;
	maxx = p.grid.maxx;
	miny = p.grid.miny;
	maxy = p.grid.maxy;
	int gride = p.grid.gride;
	int xnum = p.grid.xnum;
	int ynum = p.grid.ynum;

	Math math;
	Point end;
	Point lt = Point(minx, miny);
	Point lb = Point(minx, maxy);
	Point rt = Point(maxx, miny);
	Point rb = Point(maxx, maxy);

	if (facex == 0 && facey > 0) {
		Point xmax = math.Inter(start, face, lb, rb);
		end = Point(xmax.x, xmax.y + facey);
	}
	else if (facex == 0 && facey < 0) {
		Point xmin = math.Inter(start, face, lt, rt);
		end = Point(xmin.x, xmin.y + facey);
	}
	else if (facex > 0) {
		Point xmax = math.Inter(start, face, rt, rb);
		end = Point(xmax.x + facex, xmax.y + facey);
	}
	else if (facex < 0) {
		Point xmin = math.Inter(start, face, lt, lb);
		end = Point(xmin.x + facex, xmin.y + facey);
	}

	double length = 0;
	Point lpoint;
	int intercount = 0;
	if (start.x > end.x) {
		Point demo = start;
		start = end;
		end = demo;
	}
	int xn1 = (int)((start.x - minx) / gride);
	int xn2 = (int)((end.x - minx) / gride);
	int yn1 = (int)((start.y - miny) / gride);
	int yn2 = (int)((end.y - miny) / gride);
	if (xn1 == xn2) {
		if (yn1 > yn2) {
			yn1 = yn1 ^ yn2;
			yn2 = yn1 ^ yn2;
			yn1 = yn1 ^ yn2;
		}
		for (int j = yn1; j <= yn2; j++)
		{
			if (j > ynum) break;
			int edgepos = (xn1 >= xnum ? xnum - 1 : xn1) + (j >= ynum ? ynum - 1 : j)*xnum;
			if (edgepos < 0 || edgepos > xnum * ynum) continue;
			Cell c = p.grid.cells[edgepos];
			for (int k = 0; k < c.edges.size(); k++) {
				int eposId = c.edges[k];
				int nextId = GetNextEposId(eposId, p);
				Point p1 = p.GetPoint(eposId);
				Point p2 = p.GetPoint(nextId);
				//求交点 start->face 与 p1->p2之间的交点
				if (!math.Meet(start, end, p1, p2)) {
					continue;
				}
				intercount++;
				Point intep = math.Inter(start, end, p1, p2);
				//如果是第一次进来，则首先先给lpoint和length附值
				if (intercount == 1) {
					lpoint = intep;
					length = (intep.x - startx)*(intep.x - startx) + (intep.y - starty)*(intep.y - starty);
				}
				//求start->face 到交点的距离 
				double curLen = (intep.x - startx)*(intep.x - startx) + (intep.y - starty)*(intep.y - starty);
				//然后与length 做比较，如果更小，则记录当前的交点，否则继续下一轮。
				if (curLen < length) {
					lpoint = intep;
					length = curLen;
				}
			}
		}
	}
	else {
		double y = start.x;
		double x = start.x;
		for (int i = xn1; i <= xn2; i++)
		{
			double x3 = (i + 1) * gride + minx;
			if (x3 > end.x) x3 = end.x;
			double y3 = (start.x - x3)*(start.y - end.y) / (end.x - start.x) + start.y;

			int cur_x = (int)((x - minx) / gride);
			int cur_y = (int)((y - miny) / gride);
			int next_y = (int)((y3 - miny) / gride);
			if (cur_y > next_y) {
				cur_y = cur_y ^ next_y;
				next_y = cur_y ^ next_y;
				cur_y = cur_y ^ next_y;
			}
			for (int j = cur_y; j <= next_y; j++)
			{
				if (j > ynum) break;
				int edgepos = (cur_x >= xnum ? xnum - 1 : cur_x) + (j >= ynum ? ynum - 1 : j)*xnum;
				if (edgepos < 0 || edgepos > xnum * ynum) continue;
				Cell c = p.grid.cells[edgepos];
				for (int k = 0; k < c.edges.size(); k++) {
					int eposId = c.edges[k];
					int nextId = GetNextEposId(eposId, p);
					Point p1 = p.GetPoint(eposId);
					Point p2 = p.GetPoint(nextId);
					//求交点 start->face 与 p1->p2之间的交点
					if (!math.Meet(start, end, p1, p2)) {
						continue;
					}
					intercount++;
					Point intep = math.Inter(start, end, p1, p2);
					//如果是第一次进来，则首先先给lpoint和length附值
					if (intercount == 1) {
						lpoint = intep;
						length = (intep.x - startx)*(intep.x - startx) + (intep.y - starty)*(intep.y - starty);
					}
					//求start->face 到交点的距离 
					double curLen = (intep.x - startx)*(intep.x - startx) + (intep.y - starty)*(intep.y - starty);
					//然后与length 做比较，如果更小，则记录当前的交点，否则继续下一轮。
					if (curLen < length) {
						lpoint = intep;
						length = curLen;
					}
				}
			}
			x = x3;
			y = y3;
		}
	}
	cropoint.push_back(lpoint.x);
	cropoint.push_back(lpoint.y);

	return cropoint.data();
}

//返回 0 说明两点之间无法直达，返回 1 说明可以直达
int Path::CheckPath(double startx, double starty, double endx, double endy) {
	Point start = Point(startx, starty);
	Point end = Point(endx, endy);
	int sIndex = -1;
	int stIndex = -1;
	int eIndex = -1;
	int etIndex = -1;
	for (int i = 0; i < polygons.size(); i++) {
		double minx = polygons[i].grid.minx;
		double maxx = polygons[i].grid.maxx;
		double miny = polygons[i].grid.miny;
		double maxy = polygons[i].grid.maxy;
		if ((startx >= minx &&startx <= maxx) && (starty >= miny &&starty <= maxy)) {
			sIndex = i;
			stIndex = polygons[sIndex].FindTriangle(start);
		}
		if ((endx >= minx && endx <= maxx) && (endy >= miny&&endy <= maxy)) {
			eIndex = i;
			etIndex = polygons[eIndex].FindTriangle(end);
		}
	}
	if (sIndex != eIndex || stIndex < 0 || etIndex < 0) return 0;


	double minx, maxx, miny, maxy;
	navmesh::Polygon p = polygons[sIndex];
	minx = p.grid.minx;
	maxx = p.grid.maxx;
	miny = p.grid.miny;
	maxy = p.grid.maxy;
	int gride = p.grid.gride;
	int xnum = p.grid.xnum;
	int ynum = p.grid.ynum;

	Hash edgehashs;

	if (start.x > end.x) {
		Point demo = start;
		start = end;
		end = demo;
	}
	int xn1 = (int)((start.x - minx) / gride);
	int xn2 = (int)((end.x - minx) / gride);
	int yn1 = (int)((start.y - miny) / gride);
	int yn2 = (int)((end.y - miny) / gride);
	if (xn1 == xn2) {
		if (yn1 > yn2) {
			yn1 = yn1 ^ yn2;
			yn2 = yn1 ^ yn2;
			yn1 = yn1 ^ yn2;
		}
		for (int j = yn1; j <= yn2; j++)
		{
			if (j > ynum) break;
			int edgepos = (xn1 >= xnum ? xnum - 1 : xn1) + (j >= ynum ? ynum - 1 : j)*xnum;
			if (edgepos < 0 || edgepos > xnum * ynum) continue;
			for (int k = 0; k < p.grid.cells[edgepos].edges.size(); k++) {
				int hashk = p.grid.cells[edgepos].edges[k];
				if (edgehashs.find(hashk) == edgehashs.end()) {
					edgehashs.insert(make_pair(hashk, edgepos));
				}
			}
		}
	}
	else {
		double y = start.x;
		double x = start.x;
		for (int i = xn1; i <= xn2; i++)
		{
			double x3 = (i + 1) * gride + minx;
			if (x3 > end.x) x3 = end.x;
			double y3 = (start.x - x3)*(start.y - end.y) / (end.x - start.x) + start.y;

			int cur_x = (int)((x - minx) / gride);
			int cur_y = (int)((y - miny) / gride);
			int next_y = (int)((y3 - miny) / gride);
			if (cur_y > next_y) {
				cur_y = cur_y ^ next_y;
				next_y = cur_y ^ next_y;
				cur_y = cur_y ^ next_y;
			}
			for (int j = cur_y; j <= next_y; j++)
			{
				if (j > ynum) break;
				int edgepos = (cur_x >= xnum ? xnum - 1 : cur_x) + (j >= ynum ? ynum - 1 : j)*xnum;
				if (edgepos < 0 || edgepos > xnum * ynum) continue;
				for (int k = 0; k < p.grid.cells[edgepos].edges.size(); k++) {
					int hashk = p.grid.cells[edgepos].edges[k];
					if (edgehashs.find(hashk) == edgehashs.end()) {
						edgehashs.insert(make_pair(hashk, edgepos));
					}
				}
			}
			x = x3;
			y = y3;
		}
	}
	int i = 0;
	Math math;
	for (auto it = edgehashs.cbegin(); it != edgehashs.cend(); it++)
	{
		i++;
		int eposId = it->first;
		int nextId = GetNextEposId(eposId, p);

		Point p1 = p.GetPoint(eposId);
		Point p2 = p.GetPoint(nextId);
		//TODO:求交点 start->face 与 p1->p2之间的交点
		if (math.Meet(start, end, p1, p2)) {
			Point intep = math.Inter(start, end, p1, p2);
			//排除共点
			if ((intep.x == p1.x && intep.y == p1.y) || (intep.x == p2.x && intep.y == p2.y)
				|| (intep.x == start.x && intep.y == start.y) || (intep.x == end.x && intep.y == end.y)) {
				continue;
			}
			else {
				return 0;
			}
		}
	}

	return 1;
}

//返回多边形的格子的数据
vector<double> Path::GetGrideLine(int* length) {
	grideline.clear();
	int len = 0;
	for (int i = 0; i < polygons.size(); i++) {
		Grid g = polygons[i].grid;
		int xnum = g.xnum;
		int ynum = g.ynum;
		for (int j = 0; j <= xnum; j++) {
			Point pa1, pa2;
			double x = j * g.gride + g.minx;
			pa1.x = x;
			pa1.y = g.miny;
			pa2.x = x;
			pa2.y = g.miny + g.ynum * g.gride;
			grideline.push_back(pa1.x);
			grideline.push_back(pa1.y);
			grideline.push_back(pa2.x);
			grideline.push_back(pa2.y);
			len += 4;
		}

		for (int j = 0; j <= ynum; j++) {
			Point pa1, pa2;
			double y = j * g.gride + g.miny;
			pa1.x = g.minx;
			pa1.y = y;
			pa2.x = g.minx + g.xnum * g.gride;
			pa2.y = y;
			grideline.push_back(pa1.x);
			grideline.push_back(pa1.y);
			grideline.push_back(pa2.x);
			grideline.push_back(pa2.y);
			len += 4;
		}
	}
	*length = len;
	return grideline;
}

//返回经过三角剖分后的所有线段
vector<Line> Path::GetLines() {
	vector<Line> lines;
	for (int i = 0; i < polygons.size(); i++) {
		vector<Line> line = polygons[i].GetLines();
		lines.insert(lines.end(), line.begin(), line.end());
	}
	return lines;
}

//经过三角剖分后的所有三角数据
const int* Path::GetTriangleline(int * length)
{
	triangleline.clear();
	vector<Triangle> tri;
	int vector = 0;
	for (int i = 0; i < polygons.size(); i++) {
		tri = polygons[i].triangles;
		for (int j = 0; j < tri.size(); j++) {
			triangleline.push_back(tri[j].p1 + vector);
			triangleline.push_back(tri[j].p2 + vector);
			triangleline.push_back(tri[j].p2 + vector);
			triangleline.push_back(tri[j].p3 + vector);
			triangleline.push_back(tri[j].p3 + vector);
			triangleline.push_back(tri[j].p1 + vector);
		}
		vector += polygons[i].points.size();;
		tri.clear();
	}
	*length = triangleline.size();
	return triangleline.data();
}

Path::~Path()
{
}
