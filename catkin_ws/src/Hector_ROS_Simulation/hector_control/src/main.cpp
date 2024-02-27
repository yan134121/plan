#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"

// 写在前面 定义
// 世界坐标：方向 与机器人开机陀螺仪三轴rpy(0,0,0)时平行，原点与地面重合
// 跟随世界坐标：方向 陀螺仪三轴rpy(0,0,0)，原点在机器人躯干trunk几何中心，原点与地面的高度和设置的机器人高度有关
// 机器人自身坐标：方向 以机器人正前方为x轴正方向，与机器人同方向时左腿为y轴正方向 原点在机器人躯干trunk几何中心 原点与地面的高度和设置的机器人高度有关

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

int main(int argc, char ** argv)
{
    // 初始化一个连接外部IO设备的基类
    IOInterface *ioInter;
    // 初始化ros的节点 "hector_control"节点的名称
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    // 打印机器人名
    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;
    //创建了一个名为 ioInter 的 CheatIO类对象，用于与 ROS Gazebo 仿真通信，和与底层电机和传感器(陀螺仪世界里程计等等)和遥控器 键盘等通信，
    // CheatIO是IOInterface的派生类 包含了基类的所有public功能
    // 如果需要添加其他传感器 或新建IO设备子线程都放在这里
    ioInter = new CheatIO(robot_name);
    //表示设置频率为 1000 Hz
    ros::Rate rate(1000);
    //步长时间
    double dt = 0.001;
    //创建一个两足的(Biped)类，包含机器人一些结构偏移基本信息
    Biped biped;
    biped.setBiped();
    //创建腿部控制器，将电机的数据上传并解算，将力分解到每个电机并下发 biped类中的信息作为参数传入 所以biped类要在上面先初始化
    LegController* legController = new LegController(biped);
    //创建存储底层命令的结构体 但只有10个电机的 LowlevelCmd==>MotorCmd
    LowlevelCmd* cmd = new LowlevelCmd();
    //创建存储底层状态的结构体 包括位置 速度 角度 都是机器人在世界坐标系下
    LowlevelState* state = new LowlevelState();
    //开始设置
    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate; //状态估计结果
    //创建设置状态估计器和期望状态命令 状态估计器包括了估计机器人朝向和位置速度的组件
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);
    // 在状态估计容器中添加两个估计器来解算获得的传感器信息
    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   
    
    std::cout << "setup state etimator" << std::endl;                                                             
    //创建期望状态命令 dt = 0.001;
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);
    //创建有限状态机数据
    ControlFSMData* _controlData = new ControlFSMData;
    // 将上面所有创建的类都传入有限状态机控制器中
    _controlData->_biped = &biped; // 只为了传入hip的偏移量函数getHip2Location
    _controlData->_stateEstimator = stateEstimator;  // 传入状态估计器
    _controlData->_legController = legController; // 传入腿部控制器
    _controlData->_desiredStateCommand = desiredStateCommand; // 传入解析后的遥控器/键盘控制命令
    _controlData->_interface = ioInter;  // 传入来至io设备传感器数据
    _controlData->_lowCmd = cmd; // 传入电机的控制命令
    _controlData->_lowState = state; // 传入电机的数据
    //创建有限状态机
    FSM* _FSMController = new FSM(_controlData);
    //C++ 中的 signal 函数，用于设置在接收到中断信号（SIGINT）时执行指定的处理函数 ShutDown
    signal(SIGINT, ShutDown);
    
    while(running)
    {
        _FSMController->run();
        rate.sleep();
    }
    //最后进行一些清理工作，包括将终端设置回正常状态然后释放分配的内存
    system("stty sane");  //Terminal back to normal
    delete _controlData;
    return 0;

}
