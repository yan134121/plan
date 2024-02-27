#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf; // 四元数类

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;//状态：位置速度角速度
        Matrix<fpt,3,2> r_feet;//两只脚的位置
        Matrix<fpt,3,3> R;//身体在"跟随世界坐标系"下的旋转矩阵   由四元数得到  R = this->q.toRotationMatrix();
        Matrix<fpt,3,3> R_yaw;//机器人的位姿 绕yaw轴旋转的旋转矩阵
        Matrix<fpt,3,3> I_body;//body的转动惯量
        // Eigen库里的四元数
        Quaternionf q;//四元素
        fpt yaw;
        //fpt m = 19; // Aliengo
        fpt m = 13; // A1
    //private:
};
#endif
