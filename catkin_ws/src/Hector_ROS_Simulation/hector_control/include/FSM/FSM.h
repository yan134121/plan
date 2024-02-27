#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Passive.h"
#include "FSMState_Walking.h"
#include "../common/enumClass.h"
// 有限状态机的状态列表
struct FSMStateList{
    FSMState *invalid; //无效模式？
    FSMState_Passive *passive; //被动模式？
    FSMState_Walking *walking;

    // 在程序结束或者状态切换时释放内存
    void deletePtr(){
        delete invalid;
        delete passive;
        delete walking;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();
    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;
};

#endif