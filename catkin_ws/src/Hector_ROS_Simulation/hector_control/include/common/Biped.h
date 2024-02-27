#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include <vector>
#include "cppTypes.h"
//创建一个两足(Biped)的类 包含机器人的一些偏移量，
// 但原作者在多次修改后采用魔鬼数字而不是用这个类的，需要注意
class Biped{
  public:
    void setBiped(){
        
           mass = 13.856;
            // 以机器人躯干的几何中心为原点进行偏移，此几何中心也是机器人的坐标原点，imu原点
            // 机器人自身坐标以前进方向为x轴正方向，与机器人同方向时左腿为y轴正方向
            leg_offset_x = 0.0;
            leg_offset_y = 0.047;//0.057;
            leg_offset_z = -0.1360;//-0.125;

            leg_offset_x2 = 0.0;
            leg_offset_y2 = 0.047;//0.057;
            leg_offset_z2 = -0.136;

            hipLinkLength = 0.038; // hip offset in const.xacro髋部偏移量
            thighLinkLength = 0.22;
            calfLinkLength = 0.22;
        
    }
    int robot_index; // 1 for Aliengo, 2 for A1 选择宇树不同的机械狗电机1 代表 Aliengo，2 代表 A1
    double hipLinkLength; //髋关节链长度。
    double thighLinkLength; //大腿链长度
    double calfLinkLength; //小腿链长度
    //第一条腿的偏移值
    double leg_offset_x; 
    double leg_offset_y;
    double leg_offset_z;
    //第二条腿的偏移值
    double leg_offset_x2;
    double leg_offset_y2;
    double leg_offset_z2;
    double mass; //机器人的质量
    // 创建一个3x1的向量获取臀部位置
    Vec3<double> getHipLocation(int leg){
        //assert 语句用于确保 leg 的值在有效范围内
        assert(leg >=0 && leg <2);
        //将髋关节所有元素初始化为零
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        return pHip;
    };
    // 获取髋关节的偏移量
    Vec3<double> getHip2Location(int leg){
        assert(leg >=0 && leg <2);
        Vec3<double> pHip2 = Vec3<double>::Zero();
        if (leg == 0){
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        if (leg == 1){
            pHip2(0) = leg_offset_x2;
            pHip2(1) = -leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        return pHip2;
    };

};

#endif
