#pragma once
#ifndef SAT_COLLISION_DETECT_2D
#define SAT_COLLISION_DETECT_2D

#include <vector>
#include <iostream>
#include <stack>
#include <cmath>
#include <algorithm> // 使用C++17

using namespace std;

/// <summary>
/// 点结构体定义
/// </summary>
struct _Point
{
	double x, y;
	_Point(double x, double y) :x(x), y(y) {}
	_Point() :x(0), y(0) {}
};
using Polygon = std::vector<_Point>;

#pragma region 分离轴多边形碰撞检测

/// <summary>
/// 向量叉乘
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
double crossProduct(_Point a, _Point b);

/// <summary>
/// 向量减法
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
_Point subtract(_Point a, _Point b);

/// <summary>
/// 向量加法
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
_Point add(_Point a, _Point b);

/// <summary>
/// 向量数乘
/// </summary>
/// <param name="a"></param>
/// <param name="k"></param>
/// <returns></returns>
_Point multiply(_Point a, double k);

// 计算两点之间的距离
double distance(_Point a, _Point b);

/// <summary>
/// 计算分离轴
/// </summary>
/// <param name="polygon"></param>
/// <returns></returns>
std::vector<_Point> getAxes(const Polygon& polygon);

/// <summary>
/// 多边形投影到分离轴
/// </summary>
/// <param name="polygon"></param>
/// <param name="axis"></param>
/// <returns></returns>
std::pair<double, double> project(const Polygon& polygon, _Point axis);

/// <summary>
/// 投影重叠检测
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool overlap(std::pair<double, double> a, std::pair<double, double> b);

/// <summary>
/// 凸多边形碰撞检测
/// </summary>
/// <param name="a">第一个凸多边形</param>
/// <param name="b">第二个凸多边形</param>
/// <returns></returns>
bool polygonsIntersect(const Polygon& a, const Polygon& b);

/// <summary>
/// 计算一个点到碰撞对象多边形的距离
/// </summary>
/// <param name="p"></param>
/// <param name="polygon"></param>
/// <returns></returns>
std::tuple<double, _Point, _Point, _Point> pointToPolygonDistance(_Point p, const Polygon& polygon);

/// <summary>
/// 遍历求取最短距离
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
std::tuple<double, _Point, _Point, _Point> polygonsDistance(const Polygon& a, const Polygon& b);

#pragma endregion

#pragma region 2D凸包生成

/// <summary>
/// 任务：
/// 1. 使用Graham-Scan算法获取平面凸包点集
/// 2. 转化为FCL库的平面凸多边形点序列
/// </summary>
class Convex2D
{
public:
	/// <summary>
	/// 构造函数
	/// </summary>
	/// <param name="points">2D点集</param>
	Convex2D(const std::vector<_Point>& points);

	/// <summary>
	/// Graham扫描法求解凸包问题
	/// </summary>
	/// <returns>逆时针凸包点集</returns>
	Polygon Graham();

	/// <summary>
	/// 对凸包作三角剖分并构造与fcl库对接的Convex面序列
	/// </summary>
	/// <returns>详见fcl库的convex构造函数中的参数const std::shared_ptr<const std::vector<int>>& faces</returns>
	std::vector<int> GenFaces() = delete;

	/// <summary>
	/// 获取面的数量
	/// </summary>
	/// <returns>凸包三角面片数量</returns>
	int GetNumfaces() = delete;

private:

	/// <summary>
	/// 遍历找到左下角点
	/// </summary>
	void getPole();

	/// <summary>
	/// 坐标系原点平移到左下角点
	/// </summary>
	void DealData();

	/// <summary>
	/// 极角冒泡排序
	/// </summary>
	void PolarAngleSort();

	/// <summary>
	/// v1与v2的叉乘，v1放栈顶元素，v2放新元素
	/// </summary>
	/// <param name="v1">点向量v1</param>
	/// <param name="v2">点向量v2</param>
	/// <returns>若结果大于0，则根据右手法则v2在v1右边，认为v2是凸包的一点，放入栈顶</returns>
	double PointLine(_Point v1, _Point v2);

	/// <summary>
	/// 交换两个Point类型对象
	/// </summary>
	/// <param name="p1">点1</param>
	/// <param name="p2">点2</param>
	void Points_Swap(_Point& p1, _Point& p2);

	/// <summary>
	/// 比较a,b与x轴夹角的大小
	/// </summary>
	/// <param name="a">点a</param>
	/// <param name="b">点b</param>
	/// <returns>若a的条件>b的条件，则返回真</returns>
	bool Compare(_Point a, _Point b);

	/// <summary>
	/// vec(p1,p2)与vec(p1,p3)作叉乘，看平行四边形面积是否为0判断是否共线
	/// </summary>
	/// <param name="p1">点1</param>
	/// <param name="p2">点2</param>
	/// <param name="p3">点3</param>
	/// <returns>若p1,p2,p3共线，返回true，否则false</returns>
	bool collinear(_Point p1, _Point p2, _Point p3);

	/// <summary>
	/// 平面叉乘，用于判断
	/// </summary>
	/// <param name="v1">向量v1</param>
	/// <param name="v2">向量v2</param>
	/// <returns>若叉乘为</returns>
	double cross(_Point v1, _Point v2);

	/// <summary>
	/// 点乘
	/// </summary>
	/// <param name="v1">向量v1</param>
	/// <param name="v2">向量v2</param>
	/// <returns>v1在v2上的投影</returns>
	double dotMulti(_Point v1, _Point v2);

	/// <summary>
	/// 计算vec(p1,p2)与vec(p1,p)夹角
	/// </summary>
	/// <param name="p">新点</param>
	/// <param name="p1">栈顶点</param>
	/// <param name="p2">栈顶下一个点</param>
	/// <returns></returns>
	double getangle(_Point p, _Point p1, _Point p2);

	stack<_Point> point_stack; // 凸包点集栈，使用了Graham以后将失效，不应该在Graham以外的地方使用这个成员
	std::vector<_Point> point_list; // 逆时针保留凸包点集，point_list将代替point_stack给到其他地方使用
};
#pragma endregion

#endif // !SAT_COLLISION_DETECT_2D
