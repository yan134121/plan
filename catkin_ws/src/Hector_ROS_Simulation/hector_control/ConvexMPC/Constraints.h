#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include <eigen3/Eigen/Dense>
#include "common_types.h"
#include "RobotState.h"

/**
 * @file Constraints.h
 * @brief Constraints calculations for force-and-moment based convex MPC
 *
 * This file provides the definition for the Constraints class, which is responsible for
 * calculating the constraints needed for the convex MPC including friction cone constraints,
 * force limits, and moment limits.
 *
 */ 

class Constraints {
public:
    Constraints(RobotState& rs, const Eigen::MatrixXf& q, int horizon, int num_constraints, float motorTorqueLimit, float big_number, float f_max, const Eigen::MatrixXf& gait);

    Eigen::VectorXf GetUpperBound() const;
    Eigen::VectorXf GetLowerBound() const;
    Eigen::MatrixXf GetConstraintMatrix() const;
    
private:
    RobotState& rs_;
    int Num_Variables; // 变量的数量
    int Num_Constraints; // 约束的数量 26
    int horizon_;
    float motorTorqueLimit_; // 电机的扭矩限制
    float BIG_NUMBER_; //  极大值，5e10
    float f_max_; //  500
    Eigen::MatrixXf gait_; // 20行1列
    Eigen::MatrixXf q;     // 具有动态大小的浮点型二维矩阵
    Matrix<fpt, 3, 3> R_foot_L; // rotation matrix of left foot 左脚的旋转矩阵 脚在身体坐标系下的旋转矩阵
    Matrix<fpt, 3, 3> R_foot_R; // rotation matrix of right foot 右脚的旋转矩阵 脚在身体坐标系下的旋转矩阵

    Eigen::VectorXf U_b; // Upper bound 上界约束
    Eigen::VectorXf L_b; // Lower bound 下界约束 具有动态大小的浮点型一维向量

    Eigen::MatrixXf A_c; // Constraint matrix约束矩阵 具有动态大小的浮点型二维矩阵 A_c是一个系数矩阵

    void CalculateUpperBound();
    void CalculateLowerBound();   
    void CalculateConstarintMatrix(); 
    void CalculateFootRotationMatrix();
};

#endif //CONSTRAINT_H
