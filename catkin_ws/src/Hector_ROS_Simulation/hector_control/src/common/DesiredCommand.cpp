#include "../../include/common/DesiredCommand.h"

/**
 * @brief 归零函数
 * @details 将上一次期望状态和当前期望状态的各12个变量都清零
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.20
 */
void DesiredStateData::zero(){
    pre_stateDes = Vec12<double>::Zero();          //表示上一次的机器人的运行状态归零
    stateDes = Vec12<double>::Zero(); //当前状态归零
}

/**
 * @brief 设置状态命令,将底层的遥控器键盘等控制命令解算后传进DesiredStateCommand类
 * @details 
 * @param r roll轴
 *        p pitch轴
 *        身体的速度命令 即期望值
 *        身体的转弯命令 
 * @return 
 * @author wyt
 * @date 2023.11.20
 */
void DesiredStateCommand::setStateCommands(double r, double p, Vec3<double> v_des, double yaw_rate){
    // 只运行一次，初始化
    if(firstRun)
    {
      // 12x1的 上次期望状态矢量 中的第6个 = 状态估计中的3x1欧拉角中的yaw角
      // 如果是上电第一次运行 由状态估计器给进来
      data.pre_stateDes(5) = stateEstimate->rpy(2);          
      firstRun = false; 
    }
    // 当前各轴状态期望值 即将控制命令分解到各个轴
    data.stateDes(6) = v_des[0];       //x轴运动速度 由遥控器获取
    data.stateDes(7) = v_des[1];      //y轴运动速度 由遥控器获取
    data.stateDes(8) = 0;              //z轴运动速度 强制为零

    data.stateDes(9) = 0;              //这里应该是roll角度的持续变化率
    data.stateDes(10) = 0;             //这里应该是pitch角度的持续变化率
    data.stateDes(11) = yaw_rate;          //偏航的角度变化率

    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);    //当前质心x轴的期待位置 = 估计的x轴位置 + 0.001 x 期待的x轴速度
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);    //当前质心y轴的期待位置 = 估计的y轴位置 + 时间 x 期待的y轴速度
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);         //当前质心yaw轴的期待旋转角度  = 上次的yaw轴旋转角度 + 时间 x yaw轴的角速度

    //yaw轴角度赋值，目的是过±3.141598，将一整圈分为-3.1415~0~3.1415,
    // 在从+3.14到-3.14或-3.14到+3.14若不处理会让控制以为需要转一整圈
    if(data.stateDes(5) > 3.1 && stateEstimate->rpy(2) < 0)
    {
      data.stateDes(5) = stateEstimate->rpy(2);
    }
    if(data.stateDes(5) < -3.1 && stateEstimate->rpy(2) > 0)
    {
      data.stateDes(5) = stateEstimate->rpy(2);
    }
    //向后滚动数值，将当前的yaw轴角度赋值给上次 
    data.pre_stateDes(5) = data.stateDes(5);

     // Roll
    data.stateDes(3) = r; //机器人质心的roll角度 实际上在外面已经强制给0

    // Pitch
    data.stateDes(4) = p; //机器人质心的pitch角度 实际上在外面已经强制给0
}
// 死区，程序里没用上
double DesiredStateCommand::deadband(double command, double minVal, double maxVal){
    return (command + 1)*(maxVal-minVal)/2.0 + minVal;
}