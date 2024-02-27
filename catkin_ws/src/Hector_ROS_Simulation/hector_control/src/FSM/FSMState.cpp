#include "../../include/FSM/FSMState.h"

/**
 * @brief 有限状态机的构造函数
 * @details 给同类下的LowlevelCmd *_lowCmd赋值 即将底层电机的状态赋值进来
 * @param *data 控制有限状态机的数据
 * @return void类型的指针 有但没用，相当于没有
 * @author wyt
 * @date 2023.11.20
 */
FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr):
            _data(data), _stateName(stateName), _stateNameStr(stateNameStr)
{
    // 给同类下的LowlevelCmd *_lowCmd赋值 即将底层电机的状态赋值进来
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;
}





