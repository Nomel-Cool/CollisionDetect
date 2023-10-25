#pragma once
#ifndef SAT_COLLISION_DETECT_2D
#define SAT_COLLISION_DETECT_2D

#include <vector>
#include <iostream>
#include <stack>
#include <cmath>
#include <algorithm> // ʹ��C++17

using namespace std;

/// <summary>
/// ��ṹ�嶨��
/// </summary>
struct _Point
{
	double x, y;
	_Point(double x, double y) :x(x), y(y) {}
	_Point() :x(0), y(0) {}
};
using Polygon = std::vector<_Point>;

#pragma region ������������ײ���

/// <summary>
/// �������
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
double crossProduct(_Point a, _Point b);

/// <summary>
/// ��������
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
_Point subtract(_Point a, _Point b);

/// <summary>
/// �����ӷ�
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
_Point add(_Point a, _Point b);

/// <summary>
/// ��������
/// </summary>
/// <param name="a"></param>
/// <param name="k"></param>
/// <returns></returns>
_Point multiply(_Point a, double k);

// ��������֮��ľ���
double distance(_Point a, _Point b);

/// <summary>
/// ���������
/// </summary>
/// <param name="polygon"></param>
/// <returns></returns>
std::vector<_Point> getAxes(const Polygon& polygon);

/// <summary>
/// �����ͶӰ��������
/// </summary>
/// <param name="polygon"></param>
/// <param name="axis"></param>
/// <returns></returns>
std::pair<double, double> project(const Polygon& polygon, _Point axis);

/// <summary>
/// ͶӰ�ص����
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool overlap(std::pair<double, double> a, std::pair<double, double> b);

/// <summary>
/// ͹�������ײ���
/// </summary>
/// <param name="a">��һ��͹�����</param>
/// <param name="b">�ڶ���͹�����</param>
/// <returns></returns>
bool polygonsIntersect(const Polygon& a, const Polygon& b);

/// <summary>
/// ����һ���㵽��ײ�������εľ���
/// </summary>
/// <param name="p"></param>
/// <param name="polygon"></param>
/// <returns></returns>
std::tuple<double, _Point, _Point, _Point> pointToPolygonDistance(_Point p, const Polygon& polygon);

/// <summary>
/// ������ȡ��̾���
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
std::tuple<double, _Point, _Point, _Point> polygonsDistance(const Polygon& a, const Polygon& b);

#pragma endregion

#pragma region 2D͹������

/// <summary>
/// ����
/// 1. ʹ��Graham-Scan�㷨��ȡƽ��͹���㼯
/// 2. ת��ΪFCL���ƽ��͹����ε�����
/// </summary>
class Convex2D
{
public:
	/// <summary>
	/// ���캯��
	/// </summary>
	/// <param name="points">2D�㼯</param>
	Convex2D(const std::vector<_Point>& points);

	/// <summary>
	/// Grahamɨ�跨���͹������
	/// </summary>
	/// <returns>��ʱ��͹���㼯</returns>
	Polygon Graham();

	/// <summary>
	/// ��͹���������ʷֲ�������fcl��Խӵ�Convex������
	/// </summary>
	/// <returns>���fcl���convex���캯���еĲ���const std::shared_ptr<const std::vector<int>>& faces</returns>
	std::vector<int> GenFaces() = delete;

	/// <summary>
	/// ��ȡ�������
	/// </summary>
	/// <returns>͹��������Ƭ����</returns>
	int GetNumfaces() = delete;

private:

	/// <summary>
	/// �����ҵ����½ǵ�
	/// </summary>
	void getPole();

	/// <summary>
	/// ����ϵԭ��ƽ�Ƶ����½ǵ�
	/// </summary>
	void DealData();

	/// <summary>
	/// ����ð������
	/// </summary>
	void PolarAngleSort();

	/// <summary>
	/// v1��v2�Ĳ�ˣ�v1��ջ��Ԫ�أ�v2����Ԫ��
	/// </summary>
	/// <param name="v1">������v1</param>
	/// <param name="v2">������v2</param>
	/// <returns>���������0����������ַ���v2��v1�ұߣ���Ϊv2��͹����һ�㣬����ջ��</returns>
	double PointLine(_Point v1, _Point v2);

	/// <summary>
	/// ��������Point���Ͷ���
	/// </summary>
	/// <param name="p1">��1</param>
	/// <param name="p2">��2</param>
	void Points_Swap(_Point& p1, _Point& p2);

	/// <summary>
	/// �Ƚ�a,b��x��нǵĴ�С
	/// </summary>
	/// <param name="a">��a</param>
	/// <param name="b">��b</param>
	/// <returns>��a������>b���������򷵻���</returns>
	bool Compare(_Point a, _Point b);

	/// <summary>
	/// vec(p1,p2)��vec(p1,p3)����ˣ���ƽ���ı�������Ƿ�Ϊ0�ж��Ƿ���
	/// </summary>
	/// <param name="p1">��1</param>
	/// <param name="p2">��2</param>
	/// <param name="p3">��3</param>
	/// <returns>��p1,p2,p3���ߣ�����true������false</returns>
	bool collinear(_Point p1, _Point p2, _Point p3);

	/// <summary>
	/// ƽ���ˣ������ж�
	/// </summary>
	/// <param name="v1">����v1</param>
	/// <param name="v2">����v2</param>
	/// <returns>�����Ϊ</returns>
	double cross(_Point v1, _Point v2);

	/// <summary>
	/// ���
	/// </summary>
	/// <param name="v1">����v1</param>
	/// <param name="v2">����v2</param>
	/// <returns>v1��v2�ϵ�ͶӰ</returns>
	double dotMulti(_Point v1, _Point v2);

	/// <summary>
	/// ����vec(p1,p2)��vec(p1,p)�н�
	/// </summary>
	/// <param name="p">�µ�</param>
	/// <param name="p1">ջ����</param>
	/// <param name="p2">ջ����һ����</param>
	/// <returns></returns>
	double getangle(_Point p, _Point p1, _Point p2);

	stack<_Point> point_stack; // ͹���㼯ջ��ʹ����Graham�Ժ�ʧЧ����Ӧ����Graham����ĵط�ʹ�������Ա
	std::vector<_Point> point_list; // ��ʱ�뱣��͹���㼯��point_list������point_stack���������ط�ʹ��
};
#pragma endregion

#endif // !SAT_COLLISION_DETECT_2D
