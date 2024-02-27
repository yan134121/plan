#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

/**
 * @brief 选择控制平台
 * @details 0是gazebo仿真,1是实物操作
 * @author wyt
 * @date 2023.11.20
 */
enum class CtrlPlatform{
    GAZEBO_A1,
    REAL_A1
};

/**
 * @brief 用户命令 键盘遥控器用的
 * @details 
 * @author wyt
 * @date 2023.11.20
 */
enum class UserCommand{
    // EXIT,
    NONE,
    START,      // walking
    L2_A,       // fixedStand 固定台架
    L2_B,       // passive 被动
    L2_X,       // pushing 推
    L2_Y,       // probe 搜寻，探索
    L1_X,       // QPStand QP站立
    L1_A,      
    L1_Y       
};

/**
 * @brief 有限状态机（FSM）中的模式
 * @details 
 * @author wyt
 * @date 2023.11.20
 */
enum class FSMMode{
    NORMAL,
    CHANGE //运动模式切换过程
};

/**
 * @brief 有限状态机（FSM）中状态的名称  状态机用的
 * @details 有多种模式，但基本都没用到，可能是没开源
 * @author wyt
 * @date 2023.11.20
 */
enum class FSMStateName{
    // EXIT,
    INVALID, //initialize,初始化状态 ：无效的 站不住脚的
    PASSIVE, // 消极的，被动的
    PDSTAND, // PD站立
    QPSTAND, // QP站立
    WALKING, // 行走
    PUSHING, // 推动
    PROBE,   // 搜寻，探索
    SLAM,    // slam
};


#endif  // ENUMCLASS_H