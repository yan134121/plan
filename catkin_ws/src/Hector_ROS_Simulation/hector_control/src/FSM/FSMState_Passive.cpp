#include "../../include/FSM/FSMState_Passive.h"

// 构造函数
FSMState_Passive::FSMState_Passive(ControlFSMData *data):
                  FSMState(data, FSMStateName::PASSIVE, "passive"){}

void FSMState_Passive::enter()
{
    _data->_legController->zeroCommand();
    for(int i = 0; i < 2; i++)
    {
        // 两条腿各5个电机pd控制的d赋值 5x5的矩阵对角线赋值,使得呈现阻尼模式
        _data->_legController->commands[i].kdJoint.diagonal()<< 5, 5, 5, 5, 5;
    }

}

void FSMState_Passive::run()
{
    // 在被动模式下仍然获取底层电机的反馈参数 传感器的数据等等和遥控器命令
    _data->_legController->updateData(_data->_lowState);
    // 解算获得的传感器数据
    _data->_stateEstimator->run();
    // 更新下发的控制命令
    _data->_legController->updateCommand(_data->_lowCmd);
}

/**
 * @brief 有限状态机状态的行走状态，继承自有限状态机状态
 * @details 实现了有限状态机状态里没有的enter、run、exit函数
 * @param void
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }
}

/**
 * @brief 有限状态机状态的被动状态下的检查切换函数
 * @details 判断底层的用户命令是否是 L1_X
 * @param void 
 * @return FSMStateName 返回有限状态机状态的名字
 * @author wyt 
 * @date 2023.11.20
 */
FSMStateName FSMState_Passive::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L1_X){
        FSMStateName::WALKING;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}