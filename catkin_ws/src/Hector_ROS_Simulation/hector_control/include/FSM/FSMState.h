#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include "../common/ControlFSMData.h"
#include "../common/cppTypes.h"
#include "../common/enumClass.h"
#include "../interface/CmdPanel.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"


/**
 * @brief 有限状态机的状态基类 
 * @details 没有具体的enter、run、exit函数，checkTransition用来返回有限状态机的状态名字，检查转换的状态
 * @author wyt
 * @date 2023.11.20
 */
class FSMState
{
    public:
        FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        // 虚函数 在派生类中选择性地添加或覆盖转换检查逻辑，同时提供了一个默认的无效状态作为通用的备选项
        virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}
        // 状态名
        FSMStateName _stateName;
        // 用来打印状态名的字符串
        std::string _stateNameStr;

    protected:
        ControlFSMData *_data;
        FSMStateName _nextStateName;

        LowlevelCmd *_lowCmd;
        LowlevelState *_lowState;
        UserValue _userValue;
};

#endif // FSMSTATE_H