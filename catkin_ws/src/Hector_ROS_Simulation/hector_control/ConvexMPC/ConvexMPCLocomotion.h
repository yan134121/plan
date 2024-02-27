#ifndef CONVEXMPCLOCOMOTION_H
#define CONVEXMPCLOCOMOTION_H

#include <eigen3/Eigen/Dense>
#include "../include/common/FootSwingTrajectory.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include "GaitGenerator.h"
#include <fstream>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;

using namespace std;

/**
 * @file ConvexMPCLocomotion.h
 * @brief Convex Model Predictive Control (MPC) for Bipedal Locomotion
 *
 * This file defines the ConvexMPCLocomotion class, which implements a convex MPC
 * approach to generate and control gait patterns for bipedal robots. The class
 * offers functionality to set gait patterns, update the MPC as needed, and track 
 * the state of the robot in terms of position, orientation, and contact with the ground.
 */


struct CMPC_Result {
  LegControllerCommand commands[2];
  Vec2<float> contactPhase;
};

class ConvexMPCLocomotion {
public:
    // Constructors
    ConvexMPCLocomotion(double _dt, int _iterations_between_mpc);
  
    // Main Functionalities
    void run(ControlFSMData& data);
    // 限制步态种类在7以内,防止进入未知状态
    void setGaitNum(int gaitNum) { gaitNumber = gaitNum % 7; if(gaitNum%7 == 0) gaitNumber = 7; return; }
    bool firstRun = true;


private:
    void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode);
    void GenerateTrajectory(int* mpcTable, ControlFSMData& data, bool omniMode);

    // Locomotion and Gait Parameters
    int iterationsBetweenMPC; // MPC之间的迭代
    int horizonLength; // 规划/控制的时域长度 输入的是10
    double dt; // 0.001
    double dtMPC; // dtMPC = dt * iterationsBetweenMPC=0.001*40
    int iterationCounter = 0; // 迭代计数器
    Vec6<double> f_ff[2]; // feedforward Force 前馈力
    Vec12<double> Forces_Sol; // Forces Solution  
    Vec2<double> swingTimes; // 摆动时间
    FootSwingTrajectory<double> footSwingTrajectories[2]; // 足部摆动轨迹
    //   小跑、    跳跃、     踱步、  行走、    疾驰、      俯身、    站立;
    Gait trotting, bounding, pacing, walking, galloping, pronking, standing;

    // Feedback and Control Variables
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstSwing[2] = {true, true};
    double swingTimeRemaining[2]; // 剩余摆动时间
    double stand_traj[6]; // 站立的轨迹
    int current_gait; // 当前步态
    int gaitNumber; // 步态种类的数量
    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    Vec3<double> pFoot[2];
    CMPC_Result result;
    double trajAll[12*10];
    Mat43<double> W;  
    Vec3<double> a;  
    Vec4<double> pz;
    double ground_pitch;

    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;
    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;
    Vec3<double> pFoot_des[2];
    Vec3<double> vFoot_des[2];
    Vec3<double> aFoot_des[2];
    Vec3<double> Fr_des[2];
    Vec2<double> contact_state;
    Vec3<double> v_des_robot;
    bool climb = 0;
    ofstream foot_position;
    Vec3<double> ori_des_world;    
};


#endif //CONVEXMPCLOCOMOTION_H
