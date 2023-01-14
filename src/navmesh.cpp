//====================================
// brief: 程序的main入口文件，调用glut库来画经过三角剖分后的多边形网格和寻路的路径,主要是测试Path的方法
// author:sunxvming@163.com
// date:  2019-11-22
//====================================

#include <stdio.h>
#include <GL/glut.h>
#include <iomanip>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include "Path.h"
#include "Math.h"



double SCREEN_WIDTH = 1280;
double SCREEN_HEIGHT = 650;

double fromx = -1;
double fromy = -1;
double tox = -1;
double toy = -1;
int clicknum = 0;
int testtype = 1;

static void DrawMap() {
	printf("===========DrawMap=============\n");

	ifstream infile;
	infile.open("./map/map2.txt");  //数据格式  [childsize，parentlength, parentpoints，[0，child1length,child1points, [0，child2length, child2points]] ] 
	if (!infile) {
		printf(" unable to open myfile");
		exit(1);
	}
	string temp;
	vector<double> cont;
	vector<double> points2;
	vector<double> findpath;
	vector<double> grideline;
	vector<double> grideline2;
	vector<Line> lines;
	while (getline(infile, temp)) {
		if (temp.size() < 0) break;
		cont.push_back(atof(temp.c_str()));
	}

	int totalcount = cont.size();
	int dvector = 0;		//cont的下标
	int intnum = 0;			//所有int的个数
	int needsubnum = 0;		//需要减去的int==0的个数
	while (totalcount > 0) {
		int childsize = (int)cont[dvector++];
		int parentlength = (int)cont[dvector++];
		totalcount -= 2;

		if (childsize != 0) {
			intnum += 2;     //parentlength childsize
		}
		else {
			intnum += 1;    //parentlength
		}
		dvector += parentlength * 2;
		totalcount -= parentlength * 2;
		if (childsize == 0) {
			intnum += 1;      // childsize = 0
		}
		else {
			needsubnum += childsize;
		}
		for (int i = 0; i < childsize; i++) {
			dvector++;
			int childlength = (int)cont[dvector++];
			intnum++;
			totalcount -= 2;
			dvector += childlength * 2;
			totalcount -= childlength * 2;
		}
	}

	int charlength = sizeof(double) * (cont.size() - intnum - needsubnum) + sizeof(int) * intnum;
	char* pos = new char[charlength];    //数据格式  [parentlength, parentpoints, childsize, [child1length,child1points, [child2length, child2points]] ] 
	char* p = pos;	//存储char的初始位置
	int vector = 0; //cont的位置
	while (pos - p < charlength) {
		int childn = (int)cont[vector++];
		int size = (int)cont[vector++];

		*(int*)pos = size;  //对pos的指针指的位置赋值
		pos += sizeof(int);

		for (int i = vector; i < (size * 2 + vector); i++) {
			*(double*)pos = cont[i];
			pos += sizeof(double);
		}
		vector += size * 2;

		*(int*)pos = childn;
		pos += sizeof(int);

		if (childn > 0) {
			for (int i = 0; i < childn; i++) {
				int childsize = (int)cont[++vector];
				*(int*)pos = childsize;
				pos += sizeof(int);
				for (int j = vector + 1; j < (childsize * 2 + vector + 1); j++) {  //vector+1是因为每个多边形之间有0作为分割
					*(double*)pos = cont[j];
					pos += sizeof(double);
				}
				vector += childsize * 2;
				vector++;
			}
		}
	}
	auto path = make_shared<Path>(p, charlength);

	int pointsLen;
	int indexLen;
	int restLen;
	int triaLen;
	int findpathLen = 0;
	const double* points = path->GetPoints(&pointsLen);
	const int* indexs = path->GetIndexs(&indexLen);
	const int* triangleline = path->GetTriangleline(&triaLen);
	double minx = points[0];
	double miny = points[1];
	double maxx = points[0];
	double maxy = points[1];
	for (unsigned i = 2; i < pointsLen; i += 2)
	{
		double x = points[i];
		double y = points[i + 1];
		if (x < minx) minx = x;
		if (x > maxx) maxx = x;
		if (y < miny) miny = y;
		if (y > maxy) maxy = y;
	}
	double dx = maxx - minx;
	double dy = maxy - miny;
	double timex = SCREEN_WIDTH / dx;
	double timey = SCREEN_HEIGHT / dy;

	for (unsigned i = 0; i < pointsLen; i++)
	{
		if (i % 2 == 0) {
			points2.push_back(timex * (points[i] - minx) );  //实际坐标转换成屏幕坐标
		}
		else {
			points2.push_back(timey * (points[i] - miny) );
		}

	}
	//屏幕坐标转换成实际坐标
	float fx = fromx / timex + minx;
	float fy = fromy / timey + miny;
	float tx = tox / timex + minx;
	float ty = toy / timey + miny;
	Point from = Point(fx, fy);
	Point to = Point(tx, ty);

	const double* findpathdemo;
	const double* findcross =NULL;
	if (testtype == 1) {
		printf("fromx=%f,fromy=%f, tox=%f,toy=%f\n", fromx, fromy, tox, toy);
		findpathdemo = path->FindPaths(from, to, false, &findpathLen);
		if (findpathLen > 0) {
			for (unsigned i = 0; i < findpathLen; i++) {
				if (i % 2 == 0) {
					findpath.push_back(timex * (findpathdemo[i] - minx));
				}
				else {
					findpath.push_back(timey * (findpathdemo[i] - miny));
				}
			}
		}
	}
	else if (testtype == 2 && clicknum % 3 == 2) {
		findcross = path->FindCross(fx, fy, tx - fx, ty - fy);
		findpath.push_back(timex*(fx - minx));
		findpath.push_back(timey*(fy - miny));
		if (findcross && clicknum ) {
			findpath.push_back(timex*(findcross[0]-minx));
			findpath.push_back(timey*(findcross[1]-miny));
		}
	}
	else if (testtype == 3 && clicknum % 3 == 2) {
		int canto = path->CheckPath(fx,fy,tx,ty);
		printf("CheckPath----------%d\n",canto);
	}
	


	glLineWidth(1);//设置线段宽度
	glBegin(GL_LINES);
	////辅助的格子线
	int grideLen;
	grideline = path->GetGrideLine(&grideLen);
	for (unsigned i = 0; i < grideLen; i++) {
		if (i % 2 == 0) {
			grideline2.push_back(timex * (grideline[i] - minx) );
		}
		else {
			grideline2.push_back(timey * (grideline[i] - miny) );
		}
	}
	for (unsigned i = 0; i < grideLen; i += 4)
	{
		Line line;
		line.p1 = Point(grideline2[i], grideline2[i + 1]);
		line.p2 = Point(grideline2[i + 2], grideline2[i + 3]);
		line.color[0] = 0.8f;
		line.color[1] = 0.8f;
		line.color[2] = 0.8f;
		glColor3fv(line.color);
		glVertex2f((GLfloat)line.p1.x, (GLfloat)line.p1.y);
		glVertex2f((GLfloat)line.p2.x, (GLfloat)line.p2.y);
	}
	glEnd();

	glLineWidth(2);//设置线段宽度
	glBegin(GL_LINES);
	//画约束边和三角剖分的线
	lines = path->GetLines();
	for (unsigned i = 0; i < lines.size(); i++)
	{
		Line line = lines[i];
		glColor3fv(line.color);
		glVertex2f((GLfloat)(timex* (line.p1.x - minx)), (GLfloat)(timey* (line.p1.y - miny)));
		glVertex2f((GLfloat)(timex* (line.p2.x - minx)), (GLfloat)(timey* (line.p2.y - miny)));
	}

	//画三角形
	for (unsigned i = 0; i < triaLen; i += 2) {
		int num1 = triangleline[i];
		int num2 = triangleline[i + 1];
		Point point1 = Point(points2[num1 * 2], points2[num1 * 2 + 1]);
		Point point2 = Point(points2[num2 * 2], points2[num2 * 2 + 1]);
		Line tline;
		tline.p1 = point1;
		tline.p2 = point2;

		tline.color[0] = 1.0;
		tline.color[1] = 0.0;
		tline.color[2] = 0.0;
		//glColor3fv(tline.color);
		//glVertex2f((GLfloat)tline.p1.x, (GLfloat)tline.p1.y);
		//glVertex2f((GLfloat)tline.p2.x, (GLfloat)tline.p2.y);
	}

	//寻路
	if (findpath.size() > 0) {
		for (unsigned i = 0; i < findpath.size() - 2; i += 2) {
			Line line;
			line.p1 = Point(findpath[i], findpath[i + 1]);
			line.p2 = Point(findpath[i + 2], findpath[i + 3]);
			line.color[0] = 0.0;
			line.color[1] = 0.0;
			line.color[2] = 1.0;
			glColor3fv(line.color);
			glVertex2f((GLfloat)line.p1.x, (GLfloat)line.p1.y);
			glVertex2f((GLfloat)line.p2.x, (GLfloat)line.p2.y);

		}
	}
	glEnd();

	glPointSize(3);
	glBegin(GL_POINTS);
	for (unsigned i = 0; i < points2.size(); i += 2)
	{
		glColor3f(0.0f, 0.0, 0.0);
		glVertex3f(points2[i], points2[i + 1], 0.0);
	}
	glEnd();


}

void myDisplay(void)
{
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	DrawMap();
	glFlush();
}

// 鼠标点击的回调函数，目的为获取用户的鼠标点输入，并作为寻路的起点和终点来进行寻路
// param ：
//	button为GLUT_LEFT_BUTTON或GLUT_RIGHT_BUTTON分别表示左右按键。
//	state为按键的状态，若为按下则为GLUT_DOWN
void myClick(int button, int state, int x, int y)
{
	if (state == 1) //抬起是1
	{
		clicknum += 1;
		int t = clicknum % 3;
		if (t == 0)
		{
			fromx = fromy = tox = toy = -1;

		}
		else if (t == 1)
		{
			fromx = x;
			fromy = y;
		}
		else if (t == 2)
		{
			tox = x;
			toy = y;
		}
		printf("clicknum=%d,t=%d, x=%d,y=%d\n", clicknum, t, x, y);
		
		//if(t==2||t==0)
			//myDisplay();
	}
}

void changeTest(int i) {
	testtype = i;
}

// 右击创建菜单，用于测试不同的方法
void createMenu() {
	glutCreateMenu(changeTest); //创建菜单
	glutAddMenuEntry("test FindPaths", 1);  //在菜单中添加选项
	glutAddMenuEntry("test FindCross", 2);
	glutAddMenuEntry("test CheckPath", 3);
	glutAttachMenu(GLUT_RIGHT_BUTTON); //将菜单绑定鼠标操作
}



int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitWindowPosition(30, 30);
	glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
	glutCreateWindow("navmesh test");
	gluOrtho2D(0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
	createMenu();
	glutDisplayFunc(myDisplay);
	glutMouseFunc(myClick);
	glutMainLoop();

	return 0;
}