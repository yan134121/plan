#include "../../include/FSM/FSM.h"
#include <iostream>

/**
 * @brief FSM构造函数
 * @details 简洁而清晰直接初始化 _data 结构体
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.20
 */
FSM::FSM(ControlFSMData *data)
    :_data(data)
{

    _stateList.invalid = nullptr;
    // 实例化两个状态
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data);
    // 初始化
    initialize();
}
// 析构函数
FSM::~FSM(){
    _stateList.deletePtr();
}

// 初始化
void FSM::initialize()
{
    count = 0;
    // 将当前状态初始化为行走状态
    _currentState = _stateList.walking;
    // 通过上面的设置确定是walking还是Passive的enter函数
    _currentState -> enter();
    // 将当前状态赋值给下一状态
    _nextState = _currentState;
    // 正常模式
    _mode = FSMMode::NORMAL;
}

/**
 * @brief 有限状态机的运行函数
 * @details 先确保姿态正常再选择状态，然后
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.20
 */
void FSM::run()
{
    // 调用宇树的发送接收电机通信函数函数,获取数据
    // 因为FSM类里面包含了ControlFSMData *_data;所以直接可以调用，而ControlFSMData包含了IOInterface *_interface;
    _data->sendRecv(); 
    //当身体的轴倾斜度小于30°时属于安全范围内正常行走 !checkSafty() == 1即安全
    if(!checkSafty())
    {
        /* wyt to do:
           --> void setPassive(){cmdPanel->setPassive();} --> void setPassive(){userCmd = UserCommand::L2_B;} -->  // passive
          见名知义：在 ‘数据’ 的 ‘人机交互’ 的 ‘设置被动函数’ 的 ‘用户命令’ 的 ‘L2_B被动指令’
        */
        // 强制将用户命令置为L2_B 被动模式,不能接收到运动控制命令数据
        _data->_interface->setPassive();
    }
    // 正常模式
    if(_mode == FSMMode::NORMAL)
    {
        // 由上面的初始化FSM::initialize()知道已经第一次上电将当前状态设定为_stateList.walking状态，后面通过遥控器键盘控制转换模式
        // 所以会直接调用walking里面的run函数


        _currentState->run();//这个函数没有定义啊Liu



        // checkTransition(); --> virtual FSMStateName checkTransition() {return FSMStateName::INVALID;}
        // ‘下一个状态名字’ = ‘当前状态’ 的 ‘检查转换’ 的 ‘无效的’，
        // 选择walking状态下应该是执行walking的检查函数,而不是基类的虚函数，判断底层的用户命令是否是 L2_B，是则切换成PASSIVE，不是则WALKING保留
        _nextStateName = _currentState->checkTransition();
        // 如果：‘下一个状态名字’ 不是 ‘当前状态名字’，状态名不变那么模式就不变

        //如果下一个目标状态发生了改变，就要进入切换模式，同时获取下一个目标状态liu
        if(_nextStateName != _currentState->_stateName)
        {
            // 将模式换成 转换模式
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    // 变换模式 转入该模式后需要重启才能退出
    else if(_mode == FSMMode::CHANGE)
    {
        // std::cout << "change state" << std::endl;
        _currentState->exit();
        // 将下一状态赋值给当前状态
        _currentState = _nextState;
        // 调用passive模式下的函数
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        // 调用passive模式下的函数
        _currentState->run();       
    }

    count++;
}
/**
 * @brief FSM获取下一个状态函数
 * @details 使用状态名来切换状态
 * @param stateName 状态名
 * @return _stateList 状态列表的状态
 * @author wyt
 * @date 2023.11.20
 */
FSMState* FSM::getNextState(FSMStateName stateName)
{
    switch(stateName)
    {
        case FSMStateName::INVALID:
        // 有限状态机列表的invalid
            return _stateList.invalid;
        break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        break;
        case FSMStateName::WALKING:
            return _stateList.walking;
        break;
        default:
            return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
    /*
    数学公式：看不懂可以复制下面的代码用各种数学软件或是chatgpt看一下
       _{B}^{A}R=\begin{pmatrix}A\hat{X}_{B} & A\hat{Y}_{B} & A\hat{Z}_{B}\end{pmatrix}=\begin{pmatrix}\hat{X}_{B} & \cdot\hat{X}_{A} & \hat{Y}_{B} & \boldsymbol{\cdot}\hat{X}_{A} & \hat{Z}_{B} & {\cdot}\hat{X}_{A}\\ \hat{X}_{B} & \cdot\hat{Y}_{A} & \hat{Y}_{B} & {\cdot}Y_{A} & \hat{Z}_{B} & {\cdot}\hat{Y}_{A}\\ \hat{X}_{B} & \cdot\hat{Z}_{A} & \hat{Y}_{B} & {\cdot}Z_{A} & \hat{Z}_{B} & {\cdot}\hat{Z}_{A}\end{pmatrix}
    
    第一列表示B坐标系的X轴在A坐标系上XYZ轴的投影
    第二列表示B坐标系的Y轴在A坐标系上XYZ轴的投影
    第三列表示B坐标系的Z轴在A坐标系上XYZ轴的投影
                                     |^|  * |^|   |^|  * |^|   |^|  * |^| 
                                     |X|  * |X|   |Y|  * |X|   |Z|  * |X| 
                                     | |B * | |A  | |B * | |A  | |B * | |A

    A|      A|^|   A|^|  A|^|        |^|  * |^|   |^|  * |^|   |^|  * |^| 
     |R = (  |X|    |Y|   |Z|  ) = { |X|  * |Y|   |Y|  * |Y|   |Z|  * |Y|    }                                                                               
    B|       | |B   | |B  | |B       | |B * | |A  | |B * | |A  | |B * | |A

                                     |^|  * |^|   |^|  * |^|   |^|  * |^| 
                                     |X|  * |Z|   |Y|  * |Z|   |Z|  * |Z| 
                                     | |B * | |A  | |B * | |A  | |B * | |A

    rBody(2,2) 既表示B坐标系的Z轴在A坐标系Z轴的投影，0.5/3.14*180° = 28.66°
    因此代表的意思是 当身体的轴倾斜度小于30°时属于安全范围内正常行走
    */
    if(_data->_stateEstimator->getResult().rBody(2,2) < 0.5) 
    {
        return false;
    }
    else
    {
        return true;
    }
}