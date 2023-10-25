#include "ColiisionDetector2D.h"

#pragma region 2D͹������

void Convex2D::Points_Swap(_Point& p1, _Point& p2) {
	_Point tmp = p2;
	p2 = p1;
	p1 = tmp;
}

bool Convex2D::Compare(_Point a, _Point b)
{
	double anglea = atan2((double)a.y, (double)a.x);
	double angleb = atan2((double)b.y, (double)b.x);
	if (anglea == angleb) {//���������Ƚϣ������߷��ؽϳ���һ��
		double d1 = a.x * a.x + a.y * a.y;
		double d2 = b.x * b.x + b.y * b.y;
		return d1 > d2;
	}
	else
		return anglea > angleb;
}

bool Convex2D::collinear(_Point p1, _Point p2, _Point p3)
{
	return (p1.x * p2.y + p2.x * p3.y + p3.x * p1.y - p1.x * p3.y - p2.x * p1.y - p3.x * p2.y) == 0;
}

double Convex2D::cross(_Point v1, _Point v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

double Convex2D::dotMulti(_Point v1, _Point v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

double Convex2D::getangle(_Point p, _Point p1, _Point p2)
{
	_Point v1(p2.x - p1.x, p2.y - p1.y);
	_Point v2(p.x - p1.x, p.y - p1.y);
	double theta = atan2((double)cross(v1, v2), (double)dotMulti(v1, v2)); // Y = cross(v1,v2)=|v1||v2|sin(��), X = dot(v1,v2)=|v1||v2|cos(��), atan2(Y,X)=arctan(Y/X) �� [-��,��]
	return theta;
}

Convex2D::Convex2D(const std::vector<_Point>& points)
{
	point_list.resize(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		point_list[i] = points[i];
	}
}

vector<_Point> Convex2D::Graham()
{
	getPole();// �������ݣ���������λ

	_Point pole = point_list[0];// ����getPole����point_list[0]���Ǽ�����ϵԭ��

	DealData();//�Լ���Ϊ����ԭ�㴦���������

	PolarAngleSort();//����������

	//��ʱ����͹����㣬�����Ѿ������ȷ�p[0]��p[1]
	point_stack.push(point_list[0]);
	point_stack.push(point_list[1]);

	for (int i = 2; i < point_list.size(); i++)
	{
		if (PointLine(point_stack.top(), point_list[i]) < 0) // ��ջ������ԭ�����ߵ��ұ�
		{
			point_stack.pop();
			point_stack.push(point_list[i]);
		}
		else if (PointLine(point_stack.top(), point_list[i]) == 0) // �������ϣ�ֱ����ջ
		{
			point_stack.push(point_list[i]);
		}
		else {
			// ����µ�new_p��ջ������ԭ�����ߵ���ߣ���Ҫ����ջ��������Ԫ��p[0],p[1]��new_p�Ĺ�ϵ
			// ���vec(p[1],p[0])��vec(p[1],new_p)�ļн���[0,��]��˵��new_p��գ�new_p��͹����һ�㣬����p[1]��ѹ��new_p
			// �������[0,��]�����ҹգ���Ҫȥ��p[1]��ѹ��new_p
			_Point p1 = point_stack.top();
			point_stack.pop();
			_Point p2 = point_stack.top();
			if (getangle(point_list[i], p1, p2) <= 0) // ����˵����գ�����p1��new_p
			{
				point_stack.push(p1);
				point_stack.push(point_list[i]);
			}
			else // ����˵���ҹգ���ȥp1,����new_p������͹������ѹջ
			{
				while (getangle(point_list[i], p1, p2) > 0) // �ظ�������ջ��Ԫ���жϣ�ֱ����գ�����֤����������Ԫ�ض���͹���ڵ�
				{
					p1 = p2;
					point_stack.pop();
					p2 = point_stack.top();
				}
				point_stack.push(p1);
				point_stack.push(point_list[i]);
			}
		}
	}

	//˳ʱ���ȡջ��Ԫ��
	vector<_Point> plist;
	plist.resize(point_stack.size());
	for (int i = plist.size() - 1; i >= 0; i--)
	{
		// ����ƽ��������ϵ������ջ��Ԫ���ǰ�����ʱ��洢��
		plist[i] = _Point(point_stack.top().x + pole.x, point_stack.top().y + pole.y);
		point_stack.pop();
	}
	// ����ջ����ջ����˳ʱ��˳������plist����Ҫ��תһ��

	return plist;
}

void Convex2D::getPole()
{
	for (int i = 0; i < point_list.size(); i++)
	{
		// ȡy��С�ģ�Ҳ��������һ��ģ� || ���y��ȣ���ȡx��С�ģ�Ҳ���ǿ���һ��ģ���������һ�������µĵ�
		if (point_list[i].y < point_list[0].y || (point_list[i].y == point_list[0].y && point_list[i].x < point_list[0].x))
			Points_Swap(point_list[i], point_list[0]);
	}
}

void Convex2D::DealData()
{
	_Point p = point_list[0]; // ȡ������ԭ��
	// ������ϵԭ��ƽ�Ƶ�point_list[0]��
	for (int i = 0; i < point_list.size(); i++) {
		point_list[i].x -= p.x;
		point_list[i].y -= p.y;
	}
}

void Convex2D::PolarAngleSort()
{
	for (int i = 0; i < point_list.size() - 1; i++)
		for (int j = 0; j < point_list.size() - 1 - i; j++) {
			if (Compare(point_list[j], point_list[j + 1]))//��С����������[j+1] > [j]�򽻻�
				Points_Swap(point_list[j], point_list[j + 1]);
		}
}

double Convex2D::PointLine(_Point v1, _Point v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

#pragma endregion

#pragma region 2D͹����ײ���

double crossProduct(_Point a, _Point b)
{
	return a.x * b.y - a.y * b.x;
}

_Point subtract(_Point a, _Point b)
{
	_Point v;
	v.x = a.x - b.x;
	v.y = a.y - b.y;
	return v;
}

_Point add(_Point a, _Point b)
{
	return { a.x + b.x, a.y + b.y };
}

_Point multiply(_Point a, double k)
{
	return { a.x * k, a.y * k };
}

double distance(_Point a, _Point b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return std::sqrt(dx * dx + dy * dy);
}

std::vector<_Point> getAxes(const Polygon& polygon)
{
	std::vector<_Point> axes(polygon.size());
	for (size_t i = 0; i < polygon.size(); ++i) {
		_Point edge = subtract(polygon[i], polygon[(i + 1) % polygon.size()]);
		axes[i] = { -edge.y, edge.x };  // perpendicular to the edge
	}
	return axes;
}

std::pair<double, double> project(const Polygon& polygon, _Point axis)
{
	double min = crossProduct(axis, polygon[0]);
	double max = min;
	for (const _Point& p : polygon) {
		double projection = crossProduct(axis, p);
		min = std::min(min, projection);
		max = std::max(max, projection);
	}
	return { min, max };
}

bool overlap(std::pair<double, double> a, std::pair<double, double> b)
{
	return a.first <= b.second && a.second >= b.first;
}

bool polygonsIntersect(const Polygon& a, const Polygon& b)
{
	std::vector<_Point> axes = getAxes(a);
	for (const _Point& axis : axes) {
		if (!overlap(project(a, axis), project(b, axis))) {
			return false;  // Separating axis found
		}
	}
	axes = getAxes(b);
	for (const _Point& axis : axes) {
		if (!overlap(project(a, axis), project(b, axis))) {
			return false;  // Separating axis found
		}
	}
	return true;  // No separating axis found
}

std::tuple<double, _Point, _Point, _Point> pointToPolygonDistance(_Point p, const Polygon& polygon) {
	double minDist = std::numeric_limits<double>::max();
	_Point closestPoint;
	_Point edgeStart, edgeEnd;
	for (size_t i = 0; i < polygon.size(); ++i) {
		_Point a = polygon[i];
		_Point b = polygon[(i + 1) % polygon.size()];
		_Point ap = subtract(p, a);
		_Point ab = subtract(b, a);
		double ab2 = crossProduct(ab, ab);
		double ap_ab = crossProduct(ap, ab);
		double t = std::clamp(ap_ab / ab2, 0.0, 1.0);
		_Point closest = add(a, multiply(ab, t));
		double dist = distance(p, closest);
		if (dist < minDist) {
			minDist = dist;
			closestPoint = closest;
			edgeStart = a;
			edgeEnd = b;
		}
	}
	return { minDist, closestPoint, edgeStart, edgeEnd };
}

// �������������֮�����̾��룬����������ĵ�ͱ�
std::tuple<double, _Point, _Point, _Point> polygonsDistance(const Polygon& a, const Polygon& b) {
	double minDist = std::numeric_limits<double>::max();
	_Point closestPoint;
	_Point edgeStart, edgeEnd;
	for (const _Point& p : a) {
		auto [dist, closest, start, end] = pointToPolygonDistance(p, b);
		if (dist < minDist) {
			minDist = dist;
			closestPoint = p;
			edgeStart = start;
			edgeEnd = end;
		}
	}
	for (const _Point& p : b) {
		auto [dist, closest, start, end] = pointToPolygonDistance(p, a);
		if (dist < minDist) {
			minDist = dist;
			closestPoint = p;
			edgeStart = start;
			edgeEnd = end;
		}
	}
	return { minDist, closestPoint, edgeStart, edgeEnd };
}

#pragma endregion
