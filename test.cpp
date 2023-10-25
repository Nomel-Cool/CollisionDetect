#include <iostream>
#include <math.h>

#include "ColiisionDetector2D.h"

int main()
{
    // 人的轮廓投影点数据
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

    // 生成人轮廓2D凸包
    auto convex2d_human = c2d.Graham();

    // 探头1投影点数据
    std::vector<_Point> RawRect1Points
    {
        { -1, 1 },
        { 1,  1 },
        { 1,  2 },
        { -1,  2 }
    };
    Convex2D c2d_rect1(RawRect1Points);

    // 生成探头轮廓2D凸包
    auto convex2d_rect1 = c2d_rect1.Graham();

    // 执行碰撞检测
    bool collision_result = polygonsIntersect(convex2d_human, convex2d_rect1);

    // 距离检测
    auto min_dis = polygonsDistance(convex2d_human, convex2d_rect1);

    // 输出碰撞结果以及未碰撞时的最短距离
    if (collision_result) {
        std::cout << "Collision detected!" << std::endl;
        std::cout << "min_distance: -NaN" << std::endl;
    }
    else {
        std::cout << "No collision detected." << std::endl;
        std::cout << "min_distance:" << std::get<0>(min_dis) << std::endl;
    }

    // 探头2同理
#pragma endregion

    return 0;
}
