#ifndef WALKING_H
#define WALKING_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"

/**
 * @brief 有限状态机状态的行走状态，继承自有限状态机状态基类FSMState
 * @details 实现了有限状态机状态里没有的enter、run、exit函数
 * @author wyt
 * @date 2023.11.20
 */
class FSMState_Walking: public FSMState
{
    public:
        FSMState_Walking(ControlFSMData *data);
        ~FSMState_Walking(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
    
    private:
        ConvexMPCLocomotion Cmpc;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
};

#endif