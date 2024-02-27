#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

/**
 * @brief 有限状态机状态的被动状态，继承自有限状态机状态
 * @details 实现了有限状态机状态里没有的enter、run、exit函数
 * @author wyt
 * @date 2023.11.20
 */
class FSMState_Passive: public FSMState
{
    public:
        FSMState_Passive(ControlFSMData *data);
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition(); //检查转换
};

#endif