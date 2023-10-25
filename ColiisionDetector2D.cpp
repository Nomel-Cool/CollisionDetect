#include "ColiisionDetector2D.h"

#pragma region 2D凸包生成

void Convex2D::Points_Swap(_Point& p1, _Point& p2) {
	_Point tmp = p2;
	p2 = p1;
	p1 = tmp;
}

bool Convex2D::Compare(_Point a, _Point b)
{
	double anglea = atan2((double)a.y, (double)a.x);
	double angleb = atan2((double)b.y, (double)b.x);
	if (anglea == angleb) {//共线条件比较，若共线返回较长的一个
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
	double theta = atan2((double)cross(v1, v2), (double)dotMulti(v1, v2)); // Y = cross(v1,v2)=|v1||v2|sin(θ), X = dot(v1,v2)=|v1||v2|cos(θ), atan2(Y,X)=arctan(Y/X) ∈ [-Π,Π]
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
	getPole();// 处理数据，极点至首位

	_Point pole = point_list[0];// 经过getPole处理，point_list[0]就是极坐标系原点

	DealData();//以极点为坐标原点处理各点坐标

	PolarAngleSort();//按极角排序

	//逆时针找凸包结点，由于已经排序，先放p[0]与p[1]
	point_stack.push(point_list[0]);
	point_stack.push(point_list[1]);

	for (int i = 2; i < point_list.size(); i++)
	{
		if (PointLine(point_stack.top(), point_list[i]) < 0) // 在栈顶点与原点连线的右边
		{
			point_stack.pop();
			point_stack.push(point_list[i]);
		}
		else if (PointLine(point_stack.top(), point_list[i]) == 0) // 点在线上，直接入栈
		{
			point_stack.push(point_list[i]);
		}
		else {
			// 如果新点new_p在栈顶点与原点连线的左边，还要考虑栈顶上两个元素p[0],p[1]与new_p的关系
			// 如果vec(p[1],p[0])与vec(p[1],new_p)的夹角是[0,Π]，说明new_p左拐，new_p是凸包的一点，保留p[1]，压入new_p
			// 如果不是[0,Π]则是右拐，需要去掉p[1]，压入new_p
			_Point p1 = point_stack.top();
			point_stack.pop();
			_Point p2 = point_stack.top();
			if (getangle(point_list[i], p1, p2) <= 0) // 负角说明左拐，保留p1与new_p
			{
				point_stack.push(p1);
				point_stack.push(point_list[i]);
			}
			else // 正角说明右拐，舍去p1,由于new_p可能在凸包上遂压栈
			{
				while (getangle(point_list[i], p1, p2) > 0) // 重复这样对栈内元素判断，直到左拐，可以证明被弹出的元素都是凸包内的
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

	//顺时针获取栈内元素
	vector<_Point> plist;
	plist.resize(point_stack.size());
	for (int i = plist.size() - 1; i >= 0; i--)
	{
		// 由于平移了坐标系，而且栈内元素是按照逆时针存储的
		plist[i] = _Point(point_stack.top().x + pole.x, point_stack.top().y + pole.y);
		point_stack.pop();
	}
	// 所以栈顶到栈底是顺时针顺序，所以plist还需要反转一下

	return plist;
}

void Convex2D::getPole()
{
	for (int i = 0; i < point_list.size(); i++)
	{
		// 取y较小的（也就是下面一点的） || 如果y相等，则取x较小的（也就是靠左一点的），最终是一个靠左下的点
		if (point_list[i].y < point_list[0].y || (point_list[i].y == point_list[0].y && point_list[i].x < point_list[0].x))
			Points_Swap(point_list[i], point_list[0]);
	}
}

void Convex2D::DealData()
{
	_Point p = point_list[0]; // 取极坐标原点
	// 把坐标系原点平移到point_list[0]上
	for (int i = 0; i < point_list.size(); i++) {
		point_list[i].x -= p.x;
		point_list[i].y -= p.y;
	}
}

void Convex2D::PolarAngleSort()
{
	for (int i = 0; i < point_list.size() - 1; i++)
		for (int j = 0; j < point_list.size() - 1 - i; j++) {
			if (Compare(point_list[j], point_list[j + 1]))//从小到大排序，若[j+1] > [j]则交换
				Points_Swap(point_list[j], point_list[j + 1]);
		}
}

double Convex2D::PointLine(_Point v1, _Point v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

#pragma endregion

#pragma region 2D凸包碰撞检测

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

// 计算两个多边形之间的最短距离，并返回最近的点和边
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
