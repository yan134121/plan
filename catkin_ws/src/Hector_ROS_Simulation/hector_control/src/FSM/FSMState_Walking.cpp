#include "../../include/FSM/FSMState_Walking.h"

FSMState_Walking::FSMState_Walking(ControlFSMData *data)
                 :FSMState(data, FSMStateName::WALKING, "walking"),
                  Cmpc(0.001, 40) {}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_Walking::enter()
{
    // 身体的期望速度值 为避免跑飞进来先置零
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    // 遥控器的归零函数 最里面就是将UserValue里的 lx ly rx ry L2 vx vy 给零
     _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    // 运行两个状态估计器
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;
}

void FSMState_Walking::run()
{
    // 腿控制器中的更新数据函数  此函数返回了2条腿的运动速度，通过关节速度*雅可比矩阵得出。
    _data->_legController->updateData(_data->_lowState);         //        ControlFSMData *_data;    LowlevelState *_lowState;
    // 运行两个状态估计器，由于被实例化，所以是调不到的，直接在OrientationEstimator.cpp和PositionVelocityEstimator.cpp中找就好
    _data->_stateEstimator->run(); 
    // 获得两条腿的xy轴位移 整体的yaw角度
    _userValue = _data->_lowState->userValue;
    // 扩大范围,从原来的(-1,1)扩大到(-1.5,1.5)
    // 将左脚的y轴输入作整体的x轴输入
    v_des_body[0] = (double)invNormalize(_userValue.ly, -1.5, 1.5);
    // 将右脚的x轴输入作为整体的y轴输入
    v_des_body[1] = (double)invNormalize(_userValue.rx, -0.5, 0.5);
    // 左腿的x轴输入作为转动角度
    turn_rate = (double)invNormalize(_userValue.lx, -2.0, 2.0);
    // std::cout << "vx vy " << v_des_body[0] << " " << v_des_body[1] << std::endl;
    // 将控制命令解算成各个轴的状态
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    // 调用MPC里的函数,限制步态种类在7以内,防止进入未知状态
    Cmpc.setGaitNum(3); 
    // 运行MPC进行计算,传入各种状态包括力和位置
    Cmpc.run(*_data);
    // MPC计算完 更新下发命令
    _data->_legController->updateCommand(_data->_lowCmd);  
}

void FSMState_Walking::exit()
{      
    counter = 0; 
    _data->_interface->zeroCmdPanel();
}

/**
 * @brief 有限状态机状态的行走状态下的检查切换函数
 * @details 判断底层的用户命令是否是 L2_B
 * @param void 
 * @return FSMStateName 返回有限状态机状态的名字
 * @author wyt 
 * @date 2023.11.20
 */
FSMStateName FSMState_Walking::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::WALKING;
    }
}