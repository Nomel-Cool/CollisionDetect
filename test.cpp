#include <iostream>
#include <math.h>

#include "ColiisionDetector2D.h"

int main()
{
    // �˵�����ͶӰ������
    std::vector<_Point> RawHumanPoints
    {
        { -2, 0 },
        { -1, -1 },
        { 1, -1 },
        { 2,  0 },
        { 1,  1 },
        { -1,  1 }
    };
    Convex2D c2d(RawHumanPoints);

    // ����������2D͹��
    auto convex2d_human = c2d.Graham();

    // ̽ͷ1ͶӰ������
    std::vector<_Point> RawRect1Points
    {
        { -1, 1 },
        { 1,  1 },
        { 1,  2 },
        { -1,  2 }
    };
    Convex2D c2d_rect1(RawRect1Points);

    // ����̽ͷ����2D͹��
    auto convex2d_rect1 = c2d_rect1.Graham();

    // ִ����ײ���
    bool collision_result = polygonsIntersect(convex2d_human, convex2d_rect1);

    // ������
    auto min_dis = polygonsDistance(convex2d_human, convex2d_rect1);

    // �����ײ����Լ�δ��ײʱ����̾���
    if (collision_result) {
        std::cout << "Collision detected!" << std::endl;
        std::cout << "min_distance: -NaN" << std::endl;
    }
    else {
        std::cout << "No collision detected." << std::endl;
        std::cout << "min_distance:" << std::get<0>(min_dis) << std::endl;
    }

    // ̽ͷ2ͬ��
#pragma endregion

    return 0;
}
