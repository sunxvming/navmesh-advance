## 小demo说明
这个小demo是[导航网格寻路C++实现版(入门版)](https://github.com/sunxvming/NavMesh)的进阶版，如果你没有看那个工程的话，可以先出门左转看看那个入门版的。那个实现了一个最基本的导航网格寻路，功能简单，代码量也不多，适合没有接触或刚接触寻路功能的朋友们。
而这个是在之前的基础上又新增了一些功能，可以支持更多种类的寻路地图。如果您看过之前那个版本的代码实现的话，再看这个应该是很好看懂的。

## 新增了那些功能
1. 之前的版本一个地图是只支持一个多边形的，而且多边形不支持嵌套多边形。而这个版本的在一个地图中支持多个多边形，并且一个多边形支持嵌套多个多边形，可以映射大部分的游戏中的地图(此为主要的新增功能)
2. 检测两点是否相通，即两点都在地图内且中间没有阻挡
3. 从一个沿指定方向寻路，对应着游戏中用摇杆移动角色
4. 把经过三角剖分的地图多边形信息以二进制形式保存到文件中，方便以后直接读取并还原成Path类

## 如何编译此工程
编译的平台是windows，用的编译工具是cmake和visual studio。
你需要：
1. 下载一个windows版的cmake，然后用cmake生成visual studio的解决方案，注意生成时选32位的程序。
2. 在生成的目录中用visual studio打开名字为navmesh.sln的解决方案。
3. 编译生成项目，生成的目录在bin目录下
4. 工程中的map目录是程序读取的地图文件，你需要在navmesh.cpp中的DrawLines方法中指定读取那个地图文件 
5. 运行程序，在程序窗口点击第一下为寻路的起点，再点击第二下为寻路的终点，并进行寻路。点击第三下会把之前的路径清空。
6. 在程序窗口右击会出现一个菜单，可以选择要测试的方法

## map目录中的地图文件怎么来？
这个文件是根据美术或者地编制做的游戏地图按照一定的格式导出来的供程序用的。其中描述了地图中有几个多边形构成、构成多边形的点的位置信息。文件格式如下：
> childsize，parentlength, parentpoints，[0，child1length,child1points, [0，child2length, child2points]] 
> 内嵌多边形数量，父多边形点的数量，父多边形的点，[0，子多边形1点的数量，子多边形1的点，[0, 子多边形1点的数量，子多边形1的点]]

其中每一个上述的格式代表一个多边形，且多边形可以嵌套。而文件是有多个上述格式构成的。


## 程序截图
其中红色的线是平面多边形，你可以把它看成游戏中的地图的边界和障碍区域。
绿色的线表示经三角剖分后构成的线
蓝色的线代表寻路的路径
* 包含两个嵌套多边形的地图
![](http://www.sunxvming.com/wp-content/uploads/2019/12/微信截图_20191202105843.png)
* 根据场景资源生成的地图
![](http://www.sunxvming.com/wp-content/uploads/2019/12/微信截图_20191202104737.png)
![](http://www.sunxvming.com/wp-content/uploads/2019/12/微信截图_20191202105547.png)
![](http://www.sunxvming.com/wp-content/uploads/2019/12/微信截图_20191202105652.png)

## 主要算法
这些都可以在网上查的到，在俺的小demo中应该也很容易找的到的
* 平面多边形三角剖分(Delaunay剖分)
	+ 多边形用什么数据结构来表示
	+ 各种几何图形用什么数据结构来表示
	+ 如何生成网格以及附带数据
	+ 如何确定一个DT点
	+ 如何遍历整个多边形来完成三角剖分
* 寻路算法
* 寻路拐点算法
* 判断两条线段是否相交
* 求两条直线的交点
* 判断点是否在三角形中
* 判断点在向量的那一侧
* 已知三点求三点的夹角

## Delaunay剖分是啥？
Delaunay三角剖分其实并不是一种算法，而是一种三角剖分的标准，实现它有多种算法。它只是给出了一个“好的”三角网格的定义，它的优秀特性是空圆特性和最大化最小角特性，这两个特性避免了狭长三角形的产生，也使得Delaunay三角剖分应用广泛。
空圆特性其实就是对于两个共边的三角形，任意一个三角形的外接圆中都不能包含有另一个三角形的顶点，这种形式的剖分产生的最小角最大(比不满足空圆特性的最小角大)
 
## 主要类说明
```
// Path类，对外提供寻路的接口
class Path
	vector<Polygon> polygons;
	vector<double> points;
	vector<int > indexs;
	vector<double> finalpath;
	vector<double> cropoint;
	vector<int> restainline;
	vector<double> grideline;
	vector<int> triangleline;


// Polygon类，代表寻路多边形的类，主要实现多边形的三角剖分和寻路
Polygon
	vector<Point> points;
	vector<Triangle> triangles;
	vector<Edge> edges;
	Grid grid;
 
// 多边形对应的网格结构类，由纵横的线分割的Cell（格子）组成
Grid
	vector<Cell> cells;
	int gride;
	double minx;
	double miny;
	double maxx;
	double maxy;
	int xnum;
	int ynum;
 
//格子类，其中包含构成多边形的顶点链表和边链表
Cell
	vector<int> points;
	vector<int> edges;
 
// Triangle类，三角形的表示类
Triangle
	int p1;
	int p2;
	int p3;
	int edges[3];
	Point icenter;//重心
	Point lt;
	Point rb;
     
// Edge类，表示经过剖分后的线段，一条线段可能包含一个或两个三角，包含两个点
Edge
	int triangles[2];
	int points[2];

// Line类，画网格线的时候用到
Line
	Point p1;
	Point p2;
 
// Point类，点的表示类
Point
	double x;
	double y;
 
// Circle类，表示圆
Circle
	Point center;
	double r;
```

## 参考链接
* [平面多边形域的快速约束Delaunay三角化](https://github.com/sunxvming/navmesh/blob/master/doc/%E5%B9%B3%E9%9D%A2%E5%A4%9A%E8%BE%B9%E5%BD%A2%E5%9F%9F%E7%9A%84%E5%BF%AB%E9%80%9F%E7%BA%A6%E6%9D%9FDelaunay%E4%B8%89%E8%A7%92%E5%8C%96.pdf)
* [Nav导航网格寻路](https://blog.csdn.net/ynnmnm/article/details/44833007)
* [深入理解游戏中寻路算法](https://my.oschina.net/u/1859679/blog/1486636)