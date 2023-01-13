//====================================
// brief: Polygon类，代表寻路多边形的类，主要实现多边形的三角剖分和寻路
// author:sunxvming@163.com
// date:  2019-11-22
//====================================

#include "Polygon.h"
#include <algorithm>
using navmesh::Polygon;

Polygon::Polygon(double* pos, int32_t size)
{
	_ASSERT(size > 10);
	_ASSERT(size < MAXPOINT * 2 - 8);
	for (int32_t ii = 0; ii < size; ii += 2)
	{
		points.push_back(Point(pos[ii], pos[ii + 1]));
	}
	Gen();
}

Polygon::Polygon()
{

}

Polygon Polygon::CreateFromShort(char* cont, int32_t*len)
{
	Polygon p;
	char* cont0 = cont;

	int32_t size = *(int32_t*)cont;
	cont += sizeof(int32_t);

	for (int32_t i = 0; i < size * 2; i++)
	{
		p.points.push_back(Point(((double *)cont)[i], ((double *)cont)[i + 1]));
		i++;
	}
	cont += sizeof(double)*size * 2;
	p.pointsnum.push_back(size);

	int32_t childn = *(int32_t*)cont;
	cont += sizeof(int32_t);
	for (int32_t i = 0; i < childn; i++)
	{	
		int32_t childsize = *(int32_t*)cont;
		cont += sizeof(int32_t);
		for (int32_t j = 0; j < childsize * 2; j++)
		{
			p.points.push_back(Point(((double *)cont)[j], ((double *)cont)[j + 1]));
			j++;
		}
		cont += sizeof(double) * childsize * 2;
		p.pointsnum.push_back(childsize);
	}
	*len = cont - cont0;
	p.Gen();
	return p;
}

void Polygon::Gen()
{
	Delaunay();
	GenExtData();
}

void Polygon::CreateTriangle(Hash* eindexs, int32_t p1, int32_t p2, int32_t p3)
{
	triangles.push_back(Triangle(p1, p2, p3));
	int32_t triangle = triangles.size() - 1;
	triangles[triangle].edges[0] = CreateEdge(eindexs, triangle, p1, p2);
	triangles[triangle].edges[1] = CreateEdge(eindexs, triangle, p3, p2);
	triangles[triangle].edges[2] = CreateEdge(eindexs, triangle, p1, p3);
}

int32_t Polygon::CreateEdge(Hash* eindexs, int32_t triangle, int32_t p1, int32_t p2)
{
	int32_t k = PIndex(p1, p2);
	if (eindexs->find(k) == eindexs->end())
	{
		int32_t v = edges.size();
		edges.push_back(Edge(triangle, -1, p1, p2));
		eindexs->insert(make_pair(k, v));
		return v;
	}
	int32_t v = (*eindexs)[k];
	int32_t t2 = edges[v].triangles[1];
	_ASSERT(t2 < 0);
	edges[v].triangles[1] = triangle;
	return v;
}

//p1位于p2的右侧（位于p2的顺时针方向）
//p2位于p1左侧（位于p1的逆时针方向）
static inline int isclockwise(Point p1, Point p2)
{
	return p1.x * p2.y > p2.x* p1.y;
}

static  inline double Angle(Point cen, Point first, Point second)
{
	double dx1, dx2, dy1, dy2;

	dx1 = first.x - cen.x;
	dy1 = first.y - cen.y;

	dx2 = second.x - cen.x;

	dy2 = second.y - cen.y;

	double c = (double)sqrt(dx1 * dx1 + dy1 * dy1) * (double)sqrt(dx2 * dx2 + dy2 * dy2);

	if (c == 0) return 0;

	return (dx1 * dx2 + dy1 * dy2) / c;
}

static inline int32_t lerp(int32_t v, int32_t min, int32_t max)
{
	if (v < min)
		v = min;
	if (v > max)
		v = max;
	return v;
}

int32_t Polygon::FindDT(Grid* grid, int32_t p1, int32_t p2)
{
	double x1 = points[p1].x, y1 = points[p1].y;
	double x2 = points[p2].x, y2 = points[p2].y;
	double x = (x1 + x2) / 2, y = (y1 + y2) / 2;

	double minx = grid->minx, miny = grid->miny;
	double gride = grid->gride;
	int32_t gx = (int32_t)((x - minx) / gride);
	int32_t gy = (int32_t)((y - miny) / gride);
	int32_t xnum = grid->xnum, ynum = grid->ynum;
	int32_t d = 0;
	int32_t p3 = -1;
	double angle3 = -1;
	Point point1 = GetPoint(p1), point2 = GetPoint(p2);
	while (1)
	{
		for (int32_t i = -d; i <= d; i++)
			for (int32_t j = -d; j <= d; j++)
			{
				int32_t pos = lerp(gx + i, 0, xnum - 1) + lerp(gy + j, 0, ynum - 1)*xnum;
				if (pos >= 0 && pos < (int32_t)grid->cells.size())
				{
					Cell c = grid->cells[pos];
					for (unsigned k = 0; k < c.points.size(); k++)
					{
						int32_t p = c.points[k];
						Point point = GetPoint(p);
						if (p1 != p && p2 != p && isclockwise(point2 - point1, point - point1))  // p 要在p1p2的左侧
						{
							bool flag = JudgeIsVisible(p1, p) && JudgeIsVisible(p2, p);
							if (flag) {
								double angle = Angle(point, point1, point2);
								if (p3 == -1 || angle < angle3)
								{
									angle3 = angle;
									p3 = p;
								}
							}

						}
					}
				}
			}
		//判断是否应该结束当前趟
		if (p3 != -1)
		{
			Circle c(point1, point2, GetPoint(p3));
			Point cc = c.GetCenter();
			double radius = c.GetR();
			double l = cc.x - radius, r = cc.x + radius, t = cc.y - radius, b = cc.y + radius;
			int32_t lx = lerp((int32_t)((l - minx) / gride), 0, xnum - 1);
			int32_t rx = lerp((int32_t)((r - minx) / gride), 0, xnum - 1);
			int32_t ty = lerp((int32_t)((t - miny) / gride), 0, ynum - 1);
			int32_t by = lerp((int32_t)((b - miny) / gride), 0, ynum - 1);
			if ((gx - d) <= lx && (gx + d) >= rx && (gy - d) <= ty && (gy + d) >= by)
				break;
		}
		//_ASSERT(<);
		d++;
	}
	_ASSERT(p3 != -1);
	return p3;
}

Point Polygon::GetPoint(int32_t p)
{
	return points[p];
}

bool Polygon::IsIntersect(int32_t edgepos, int32_t pa1, int32_t p1) {
	Point pa = GetPoint(pa1);
	Point p = GetPoint(p1);
	Math math;
	Cell c = grid.cells[edgepos];
	bool flag = false;
	for (unsigned m = 0; m < c.edges.size(); m++) {
		int32_t eposId = c.edges[m];
		int32_t next_eposId = 0;
		int32_t sum = 0;
		for (int32_t i = 0; i < pointsnum.size(); i++) {
			sum += pointsnum[i];

			if (eposId == sum - 1) {
				next_eposId = eposId - (pointsnum[i] - 1);
				break;
			}
			else if (eposId < sum - 1) {
				next_eposId = eposId + 1;
				break;
			}
			else if (eposId > sum - 1) {
				continue;
			}
		}

		flag = math.Meet(GetPoint(next_eposId), GetPoint(eposId), pa, p);
		if ((eposId == pa1) || (eposId == p1) || (next_eposId == pa1) || (next_eposId == p1))
		{
			flag = false;
		}
		if (flag)
			return true; 
	}
	return false;
}

//false代表pa 与p不可见，true代表可见
bool Polygon::JudgeIsVisible(int pindex1, int pindex2) {
	Point p1 = GetPoint(pindex1);
	Point p2 = GetPoint(pindex2);
	//检查最大最小值合法性
	Point p0 = points[0];
	double minx = p0.x, miny = p0.y, maxx = p0.x, maxy = p0.y;
	for (auto it = points.cbegin() + 1; it != points.cend(); it++)
	{
		double x = it->x;
		double y = it->y;
		if (x < minx) minx = x;
		if (x > maxx)maxx = x;
		if (y < miny) miny = y;
		if (y > maxy)maxy = y;
	}

	double dx = maxx - minx, dy = maxy - miny;
	int gride = (int)sqrt(dx * dy / (points.size()));
	int xnum = (int)ceil(dx / gride);
	int ynum = (int)ceil(dy / gride);

	if (p1.x > p2.x) {
		Point demo = p2;
		p2 = p1;
		p1 = demo;
	}
	int xn1 = (int)((p1.x - minx) / gride);
	int yn1 = (int)((p1.y - miny) / gride);
	int xn2 = (int)((p2.x - minx) / gride);
	int yn2 = (int)((p2.y - miny) / gride);

	if (xn1 == xn2) {
		if (yn1 > yn2) {
			yn1 = yn1 ^ yn2;
			yn2 = yn1 ^ yn2;
			yn1 = yn1 ^ yn2;
		}
		for (int j = yn1; j <= yn2; j++)
		{
			if (j > ynum) break;
			int edgepos = (xn1 >= xnum ? xnum - 1 : xn1) + (j >= ynum ? ynum - 1 : j) * xnum;
			if (IsIntersect( edgepos, pindex1, pindex2))
				return false;
		}
	}
	else {
		double x = p1.x;
		double y = p1.y;
		for (int i = xn1; i <= xn2; i++)
		{
			double x3 = (i + 1) * gride + minx;
			if (x3 > p2.x) x3 = p2.x;
			// 两点求得方程 (y-y2)/(y1-y2)=(x-x2)/(x1-x2)，把x3代入计算y3
			double y3 = (p2.x - x3) * (p2.y - p1.y) / (p1.x - p2.x) + p2.y;

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
				int edgepos = (cur_x >= xnum ? xnum - 1 : cur_x) + (j >= ynum ? ynum - 1 : j) * xnum;
				if (IsIntersect(edgepos, pindex1, pindex2))
					return false;
			}
			x = x3;
			y = y3;
		}
	}
	return true;
}


void Polygon::Delaunay()
{
	//检查最大最小值合法性
	Point p0 = points[0];
	double minx = p0.x, miny = p0.y, maxx = p0.x, maxy = p0.y;
	for (auto it = points.cbegin() + 1; it != points.cend(); it++)
	{
		double x = it->x;
		double y = it->y;
		if (x < minx) minx = x;
		if (x > maxx)maxx = x;
		if (y < miny) miny = y;
		if (y > maxy)maxy = y;
	}

	double dx = maxx - minx, dy = maxy - miny;
	int32_t gride = (int32_t)sqrt(dx*dy / (points.size()));
	int32_t xnum = (int32_t)ceil(dx / gride);
	int32_t ynum = (int32_t)ceil(dy / gride);
	vector<Cell> cells(xnum*ynum);
	//把点和边都放到格子里
	for (auto it = points.cbegin(); it != points.cend(); it++)
	{
		//point放到Grid里
		double x = it->x;
		double y = it->y;
		int32_t xn = (int32_t)((x - minx) / gride);
		int32_t yn = (int32_t)(((y - miny) / gride));
		int32_t pos = (xn >= xnum ? xnum - 1 : xn) + (yn >= ynum ? ynum - 1 : yn)*xnum;
		int32_t point = it - points.cbegin();
		cells[pos].points.push_back(point);
		
		//edges放到Grid里
		Point p = GetPoint(point);
		int32_t p1num = 0;
		int32_t sum = 0;
		for (int32_t i = 0; i < pointsnum.size(); i++) {
			
			sum += pointsnum[i];
			if(point == sum-1){   //如果是多边形的最后一个点
				p1num = point - (pointsnum[i] - 1);
				break;
			}
			else if(point < sum -1){
				p1num = point + 1;
				break;
			}
			else if (point > sum - 1) {   //点不在这个多边形中
				continue;
			}
		}
		
		Point p1 = GetPoint(p1num);
		if (p.x > p1.x) {
			Point demo = p;
			p = p1;
			p1 = demo;
		}
		int32_t xn1 = (int32_t)((p.x - minx) / gride);
		int32_t xn2 = (int32_t)((p1.x - minx) / gride);
		int32_t yn1 = (int32_t)((p.y - miny) / gride);
		int32_t yn2 = (int32_t)((p1.y - miny) / gride);

		if (xn1 == xn2) {
			if (yn1 > yn2) {
				yn1 = yn1 ^ yn2;
				yn2 = yn1 ^ yn2;
				yn1 = yn1 ^ yn2;
			}
			for (int32_t j = yn1; j <= yn2; j++)
			{
				if (j > ynum) break;
				int32_t edgepos = (xn1 >= xnum ? xnum - 1 : xn1) + (j >= ynum ? ynum - 1 : j)*xnum;
				cells[edgepos].edges.push_back(point);
			}
		}
		else {
			//将x值偏小的 y值保存起来
			double y = p.y;
			double x = p.x;
			int32_t cur_x = (int32_t)((x - minx) / gride);
			int32_t cur_y = (int32_t)((y - miny) / gride);
			for (int32_t i = xn1; i <= xn2; i++)
			{
				double x3 = (i + 1) * gride + minx;
				if (x3 > p1.x) x3 = p1.x;
				// 两点求得方程 (y-y2)/(y1-y2)=(x-x2)/(x1-x2)，把x3代入计算y3
				double y3 = (p.x - x3)*(p.y - p1.y) / (p1.x - p.x) + p.y;
				int32_t next_y = (int32_t)((y3 - miny) / gride);
				if (cur_y > next_y) {
					cur_y = cur_y ^ next_y;
					next_y = cur_y ^ next_y;
					cur_y = cur_y ^ next_y;
				}
				for (int32_t j = cur_y; j <= next_y; j++)
				{
					if (j > ynum) break;
					int32_t edgepos = (cur_x >= xnum ? xnum - 1 : cur_x) + (j >= ynum ? ynum - 1 : j)*xnum;
					cells[edgepos].edges.push_back(point);
				}
				x = x3;
				y = y3;
				cur_x++;
				cur_y = next_y;
			}
		}
		
	}

	grid.cells = cells;
	grid.gride = gride;
	grid.minx = minx;
	grid.maxx = maxx;
	grid.miny = miny;
	grid.maxy = maxy;
	grid.xnum = xnum;
	grid.ynum = ynum;

	Hash eindexs;    //k->v (Pindex->edges的index)，存放着已经处理过的所有三角形的边 
	
	//Hash restrains;   //把所有约束边放到里面
	for (unsigned i = 0; i < points.size(); i++)
	{
		int32_t num1 = i;
		int32_t sum = 0;
		int32_t num2 = 0;
		for (int32_t i = 0; i < pointsnum.size(); i++) {
			sum += pointsnum[i];

			if (num1 == sum - 1) {  //多边形的最后一个点
				num2 = num1 - (pointsnum[i] - 1);
				break;
			}
			else if (num1 < sum - 1) {
				num2 = num1 + 1;
				break;
			}
			else if (num1 > sum - 1) {   //点不在这个多边形中
				continue;
			}
		}
		restrains.insert(make_pair((int32_t)PIndex(num1, num2), 1));
	}
	vector<int32_t> es;
	int32_t p1 = 0, p2 = 1;
	int32_t e = PIndex(p1, p2);
	while (1)
	{
		int32_t p3 = FindDT(&grid, p1, p2);
		if (restrains.find((int32_t)PIndex(p1, p3)) == restrains.end())
		{
			vector<int32_t>::iterator it = std::find(es.begin(), es.end(), PIndex(p1, p3));
			if (it == es.end())
			{
				es.push_back(PIndex(p1, p3));
			}
			else {
				es.erase(it);
			}
		}
		if (restrains.find((int32_t)PIndex(p2, p3)) == restrains.end())
		{
			vector<int32_t>::iterator it = std::find(es.begin(), es.end(), PIndex(p2, p3));
			if (it == es.end())
			{
				es.push_back(PIndex(p2, p3));
			}
			else {
				es.erase(it);
			}
		}
		CreateTriangle(&eindexs, p1, p2, p3);
		if (es.empty()) { 
			break; 
		}
		e = *(es.end() - 1);  //出栈
		es.pop_back();
		int32_t* points = edges[eindexs[e]].points;
		p1 = points[0], p2 = points[1];
	}
}

void Polygon::GenExtData()
{
	for (unsigned i = 0;  i < triangles.size(); i++)
	{
		Triangle t = triangles[i];
		t.GenExtData(this);
		triangles[i] = t;
	}

	for (unsigned i = 0; i < edges.size(); i++)
	{
		Edge e = edges[i];
		int32_t e0 = e.triangles[0];
		int32_t e1 = e.triangles[1];
		if (e0 >= 0 && e1 >= 0) {  //一条边有两个三角形的
			Triangle t0 = triangles[e0];
			Triangle t1 = triangles[e1];
			int32_t p0 = e.points[0], p1 = e.points[1];
			//使其在第一个三角中，让p1位于p0的左侧（点的位置固定起来，p1是左点，p0是右点），使寻路时拐点算法的两边方向都是一致的
			//0的位置放右点
			if (!isclockwise(GetPoint(p0) - t0.icenter, GetPoint(p1) - t0.icenter))
			{
				e.points[0] = p1;
				e.points[1] = p0;
				edges[i] = e;
			}
		}
	}
}

vector<Line> Polygon::GetLines() {
	vector<Line> lines(edges.size());
	for (unsigned i = 0; i < edges.size(); i++)
	{
		Line line;
		line.p1 = GetPoint(edges[i].points[0]);
		line.p2 = GetPoint(edges[i].points[1]);
		if (edges[i].IsRestrain(this))
		{
			line.color[0] = 1.0;
			line.color[1] = 0.0;
			line.color[2] = 0.0;
		}
		else {
			line.color[0] = 0.0;
			line.color[1] = 1.0;
			line.color[2] = 0.0;
		}
		lines[i] = line;
	}
	return lines;
}

vector<Line> Polygon::GetGrideLines() {
	vector<Line> lines(grid.ynum + grid.xnum + 2);
	for (int32_t i = 0; i <= grid.xnum; i++)
	{
		Line line;
		Point pa1, pa2;
		double x = i*grid.gride + grid.minx;
		pa1.x = x;
		pa1.y = grid.miny;
		pa2.x = x;
		pa2.y = grid.miny + grid.ynum * grid.gride;
		line.p1 = pa1;
		line.p2 = pa2;

		line.color[0] = 0.2f;
		line.color[1] = 0.3f;
		line.color[2] = 0.3f;

		lines[i] = line;
	}

	for (int32_t j = 0; j <= grid.ynum; j++)
	{
		Line line;
		Point pa1, pa2;
		double y = j * grid.gride + grid.miny;
		pa1.x = grid.minx;
		pa1.y = y;
		pa2.x = grid.minx + grid.xnum * grid.gride;
		pa2.y = y;
		line.p1 = pa1;
		line.p2 = pa2;

		line.color[0] = 0.2f;
		line.color[1] = 0.3f;
		line.color[2] = 0.3f;

		lines[j + grid.xnum + 1] = line;
	}
	return lines;
}

vector<Point> Polygon::GetCenters() {
	vector<Point> centers(triangles.size());
	for (unsigned i = 0; i < triangles.size(); i++)
	{
		centers[i] = triangles[i].icenter;
	}
	return centers;
}

int32_t Polygon::FindTriangle(Point p)
{
	for (unsigned i = 0; i < triangles.size(); i++)
	{
		if (triangles[i].Contain(this, p)) return i;
	}
	return -1;
}

#define distance( p1, p2 )((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))



double Polygon::Distance(Point from, int32_t edge, int32_t tindex, Point to)
{
	int32_t p0 = edges[edge].points[0];
	int32_t p1 = edges[edge].points[1];

	Point e0 = GetPoint(p0);
	Point e1 = GetPoint(p1);
	Point ecenter = Point((e0.x + e1.x) / 2, (e0.y + e1.y) / 2);

	int32_t p2 = triangles[tindex].p3;
	if (p2 == p0 || p2 == p1) p2 = triangles[tindex].p2;
	if (p2 == p0 || p2 == p1) p2 = triangles[tindex].p1;

	Point e2 = GetPoint(p2);
	Point ecenter2;
	if(distance(e0, to) < distance(e1, to))
	{
		ecenter2.x = (e0.x + e2.x) / 2;
		ecenter2.y = (e0.y + e2.y) / 2;
	}
	else {
		ecenter2.x = (e1.x + e2.x) / 2;
		ecenter2.y = (e1.y + e2.y) / 2;
	}
	return distance(from, ecenter) + distance(ecenter, ecenter2) + distance(ecenter2, to);
}

//true时为中点寻路，false为拐点寻路
vector<Point> Polygon::FindPath(Point from, Point to, bool isturn)
{
	int32_t tfrom = FindTriangle(from);
	int32_t tto = FindTriangle(to);
	vector<int32_t> ts;
	vector<int32_t> es;
	vector<Point> ways;
	if (tfrom < 0) return ways;
	if (tto < 0) return ways;
	ways.push_back(from);
	if (tfrom == tto)
	{
		ways.push_back(to);
		return ways;
	}
	ts.push_back(tfrom);
	//A*
	int32_t start = tfrom;
	Hash visited;
	visited.insert(make_pair(tfrom, 1));
	Point startp = from;
	while (!ts.empty())
	{
		start = *(ts.cend() - 1);
		double weight = -1;
		int32_t next = -1;
		int32_t e;
		for (int32_t i = 0; i < 3; i++)
		{
			int32_t eindex = triangles[start].edges[i];
			Edge edge = edges[eindex];
			//找到非约束边的，非原三角形的另外其他的三角形
			int32_t nextt = edge.triangles[0] == start ? edge.triangles[1] : edge.triangles[0];
			if (nextt >= 0)  // 非约束边的
			{
				if (nextt == tto)
				{
					next = tto;
					e = eindex;
					break;
				}
				if (visited.find(nextt) == visited.end())
				{
					Point e1 = GetPoint(edge.points[0]);
					Point e2 = GetPoint(edge.points[1]);
					Point ecenter = Point((e1.x + e2.x) / 2, (e1.y + e2.y) / 2);
					double d = Distance(startp, eindex, nextt, to);
					if (next < 0 || weight > d) next = nextt, startp = ecenter, weight = d, e = eindex;
				}
			}
		}
		if (next == tto)
		{
			ts.push_back(next);
			es.push_back(e);
			break;
		}
		else if (next >= 0)
		{
			visited.insert(make_pair(next, 1));
			ts.push_back(next);
			es.push_back(e);
		}
		else {
			ts.pop_back();
			es.pop_back();
		}
	}

	if (ts.empty()) return ways;
	int32_t t = 0;
	int32_t size = es.size();
	if (isturn) {
		//中点寻路算法
		for (int32_t i = 0; i < es.size(); i++) {
			Edge e = edges[es[i]];
			Point p1 = GetPoint(e.points[0]);
			Point p2 = GetPoint(e.points[1]);
			Point cp = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
			ways.push_back(cp);
		}
		ways.push_back(to);
	}
	else {
		while (t < size)
		{
			Point p = *(ways.cend() - 1);  //循环开始点是起点
			Edge e = edges[es[t]];
			int ep0, ep1;
			//ts[t]表示当前要处理的三角
			if (ts[t] == e.triangles[0])  //判断是否是第0个三角，第0个三角p1是在p0左侧的
			{
				ep0 = e.points[0], ep1 = e.points[1];
			}
			else {   //是第一个三角的话，p1在p0的右侧，需要换下顺序
				ep0 = e.points[1], ep1 = e.points[0];
			}

			Point p1 = GetPoint(ep0), p2 = GetPoint(ep1);   // p1是右点，p2是左点
			Point line1 = p1 - p, line2 = p2 - p;           //line1右线， line2左线
			int i = t + 1;
			int t1 = i, t2 = i;
			//每一次循环就是要找到一个拐点，然后下一次再从这个拐点找下一个拐点
			bool haspoint = false;
			for (; i < size; i++)
			{
				Edge nxte = edges[es[i]];
				int nep0, nep1;
				if (ts[i] == nxte.triangles[0])
				{
					nep0 = nxte.points[0], nep1 = nxte.points[1];
				}
				else {
					nep0 = nxte.points[1], nep1 = nxte.points[0];
				}
				Point np1 = GetPoint(nep0), np2 = GetPoint(nep1);
				if (!isclockwise(line1, np2 - p)) {//np2（左点）不在line1（右线）的左侧的话，直接确定p1为拐点
					Point pp = GetPoint(ep0);
					p = Point(pp.x + 0, pp.y - 0);
					ways.push_back(Point(pp.x + 0, pp.y - 0));
					t1++;
					t = nep0 == ep0 ? (i++, t1 + 1) : t1;
					haspoint = true;
					break;
				}
				else if (isclockwise(line1, np1 - p)) //np1（右点）在line1（右线）的左侧的话，更新line1
				{
					line1 = np1 - p;
					ep0 = nep0;
					t1 = i;
				}

				if (isclockwise(line2, np1 - p)) { //np1（右点）在line2（左线）的左侧的话，直接确定拐点
					Point pp = GetPoint(ep1);
					p = Point(pp.x + 0, pp.y - 0);
					ways.push_back(Point(pp.x + 0, pp.y - 0));
					t2++;
					t = nep1 == ep1 ? (i++, t2 + 1) : t2;
					haspoint = true;
					break;
				}
				else if (!isclockwise(line2, np2 - p))  //np2（左点）不在line2（左线）的左侧的话，更新line2
				{
					line2 = np2 - p;
					ep1 = nep1;
					t2 = i;
				}

			}
			if (i >= size - 1)  //左线和右线一直更新更新到了最后
			{
				if (!isclockwise(line1, to - p))  //终点在line1（右线）右侧
				{
					ways.push_back(GetPoint(ep0));    //左点为拐点
					t = t1;
				}
				else if (isclockwise(line2, to - p))   //终点在line2（左线）左侧
				{
					ways.push_back(GetPoint(ep1));
					t = t2;
				}
				if (t >= size - 1)          // //终点在中间
				{
					ways.push_back(to);
				}
				t++;
			}
		}
	}//End else
	return ways;
}

void Polygon::Save(FILE* file) {
	int32_t pointsLen = points.size();
	fwrite(&(pointsLen), sizeof(int32_t), 1, file);
	fwrite(points.data(), sizeof(Point), pointsLen, file);
	
	int32_t trianglesLen = triangles.size();
	fwrite(&(trianglesLen), sizeof(int32_t), 1, file);
	fwrite(triangles.data(), sizeof(Triangle), trianglesLen, file);

	int32_t edgeLen = edges.size();
	fwrite(&(edgeLen), sizeof(int32_t), 1, file);
	fwrite(edges.data(), sizeof(Edge), edgeLen, file);

	int32_t pLen = pointsnum.size();
	fwrite(&(pLen), sizeof(int32_t), 1, file);
	fwrite(pointsnum.data(), sizeof(int32_t), pLen, file);
	
	int32_t cellsize = grid.xnum * grid.ynum;
	fwrite(&(cellsize), sizeof(int32_t), 1, file);
	for (int32_t i = 0; i < cellsize; i++) {
		int32_t poLen = grid.cells[i].points.size();
		fwrite(&(poLen), sizeof(int32_t), 1, file);
		fwrite(grid.cells[i].points.data(), sizeof(int32_t), poLen, file);

		int32_t edLen = grid.cells[i].edges.size();
		fwrite(&(edLen), sizeof(int32_t), 1, file);
		fwrite(grid.cells[i].edges.data(), sizeof(int32_t), edLen, file);
	}
	fwrite(&(grid.gride), sizeof(int32_t), 1, file);
	fwrite(&(grid.minx), sizeof(double), 1, file);
	fwrite(&(grid.maxx), sizeof(double), 1, file);
	fwrite(&(grid.miny), sizeof(double), 1, file);
	fwrite(&(grid.maxy), sizeof(double), 1, file);
	fwrite(&(grid.xnum), sizeof(int32_t), 1, file);
	fwrite(&(grid.ynum), sizeof(int32_t), 1, file);
}

Polygon::~Polygon()
{

}
