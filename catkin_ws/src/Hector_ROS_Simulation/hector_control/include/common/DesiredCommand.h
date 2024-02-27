/*!
 * @file DesiredCommand.h
 * @brief convert keyborad/gamepad command into desired
 * tracjectory for the robot
 */ 

#ifndef DESIREDCOMMAND_H
#define DESIREDCOMMAND_H

#include "cppTypes.h"
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include "StateEstimatorContainer.h"
#include "../interface/CmdPanel.h"

struct DesiredStateData{

    DesiredStateData() { zero(); }

    // Zero all data 归零函数
    void zero();

    // Instataneous desired state comman即时所需状态命令
    /*
      00 x轴位移
      01 y轴位移
      02 z轴位移
      03 roll轴角度
      04 pitch轴角度
      05 yaw轴角度
      06 x轴运动速度
      07 y轴运动速度
      08 z轴运动速度
      09 roll角度的变化率
      10 pitch角度的变化率
      11 yaw轴的角度变化率
    */
    Vec12<double> stateDes;
    Vec12<double> pre_stateDes;

    int mode;
};
//期望状态命令
class DesiredStateCommand {
  public:
    // Initialize， 构造函数初始化 将状态估计器,步长时间赋值进来
    DesiredStateCommand(StateEstimate* _estimate, double _dt){
      stateEstimate = _estimate;
      dt = _dt; // 0.001 在main函数里给进来
    }
    // 虚函数 转换为状态命令  没有实函数
    void convertToStateCommands(UserValue _userValue);
    // 设置状态命令 在运动状态里调用
    void setStateCommands(double r, double p, Vec3<double> v_des, double yaw_rate);
    // 虚函数 设置模式 没用上
    void setmode(int ctrl_mode) {data.mode = ctrl_mode;}
    // 死区 没用上
    double deadband(double command, double minVal, double maxVal);
    // These should come from the inferface    
    //来自上位机的限幅设置，这里只是初始化
    bool firstRun = true;
    // 没用上
    double maxRoll = 0.4;
    double minRoll = -0.4;
    double maxPitch = 0.4;
    double minPitch = -0.4;
    double maxVelX = 2.0;
    double minVelX = -2.0;
    //double maxVelX = 5.0;
    //double minVelX = -5.0;
    double maxVelY = 0.5;
    double minVelY = -0.5;
    //转速限幅
    double maxTurnRate = 2.0; 
    double minTurnRate = -2.0;
    // 期望状态数据
    DesiredStateData data;

    //~DesiredStateCommand();
  private:
    StateEstimate* stateEstimate; //状态估计

    double dt; // Control loop time step
};



#endif