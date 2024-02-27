#include "../../include/interface/CheatIO.h"
#include "../../include/interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
// ros仿真用的结束函数
inline void RosShutDown(int sig){
    ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}
// 接受robot_name 并从 IOInterface 类继承
CheatIO::CheatIO(std::string robot_name):IOInterface()
{
    // int argc; char **argv;
    // ros::init(argc, argv, "unitree_gazebo_servo");
    //输出：具有欺骗状态的ROS Gazebo仿真控制界面
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;
    _robot_name = robot_name;

    // start subscriber 启动用户 初始化接收数据
    initRecv();
    //创建一个异步的ROS消息处理器，并启动它，这通常用于在独立的线程中处理ROS消息
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(3000);     //wait for subscribers start延时3000毫秒 等待订阅者启动
    // initialize publisher 初始化发送数据
    initSend();   
    //注册一个信号处理函数
    signal(SIGINT, RosShutDown);
    //新建键盘输入的类对象，在该类里新建接收键盘的子线程
    cmdPanel = new KeyBoard();
}

//析构函数
CheatIO::~CheatIO()
{
    ros::shutdown();
}

/**
 * @brief 发送接收电机通信函数
 * @details 将电机的状态通过ros发布出去,上层的控制状态和姿态传至低层级的控制状态中
 * @param cmd 命令  
 *        state 状态 
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);                    //将电机的状态（电机位置、速度、转矩和控制参数）通过ros发布出去
    recvState(state);                //将上层的控制状态和姿态传至低层级的控制状态中，并保存起来。


    // 虚拟函数 更新速度命令 实际没有写
    cmdPanel->updateVelCmd(state);   
    // 简单包装变量,以函数的形式传递,避免数据乱飞
    state->userCmd = cmdPanel->getUserCmd();     //获得操作者（遥控器）的指令
    state->userValue = cmdPanel->getUserValue();  //获得使用者的值

}

void CheatIO::sendCmd(const LowlevelCmd *cmd)
{
    for(int i = 0; i < 10; i++){
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A = 10（闭环伺服控制）
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;//电机的位置（关节位置），用于指定电机在关节空间中的目标位置。
        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;//电机的速度（关节速度），用于指定电机在关节空间中的目标速度。
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;//电机的力矩（关节力矩），用于指定电机在关节空间中的目标扭矩或力矩。
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;//控制器微分系数
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;//控制器比例系数
    }
    for(int m = 0; m < 10; m++){
        // 将一个消息发布到 ROS 中的特定话题，以便其他节点可以订阅并处理这些消息。
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    // 在主循环中运行一次，处理已经到达的消息，并调用相应的回调函数
    ros::spinOnce();
}

//用于接收传入的 LowlevelState 类型的状态信息，并将其存储到类的成员变量中。
void CheatIO::recvState(LowlevelState *state)
{
    for(int i = 0; i < 10; i++)           //电机的信息从上层状态传至底层状态
    {
        state->motorState[i].q = _highState.motorState[i].q;//_highState属于HighState类
        state->motorState[i].dq = _highState.motorState[i].dq;
        state->motorState[i].tauEst = _highState.motorState[i].tauEst;
    }
    for(int i = 0; i < 3; i++){
        state->imu.quaternion[i] = _highState.imu.quaternion[i];//imu的四元数信息
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];//imu中的陀螺仪信息
        state->position[i] = _highState.position[i];//
        state->vWorld[i] = _highState.velocity[i];//这里的HighState在哪里有定义？
    }
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
}

// 创建十个发布者对应十个不同的关节控制器 向ROS中的Gazebo仿真环境发送关节命令
void CheatIO::initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/L_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/L_hip2_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/L_thigh_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/L_calf_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/L_toe_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/R_hip_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/R_hip2_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/R_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/R_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>( "/" + _robot_name + "_gazebo/R_toe_controller/command", 1);
}
// 创建一个订阅者 用于接收Gazebo仿真环境的模型状态消息
// 创建十个订阅者对应十个不同的关节控制器。用于接收关节控制器的状态消息。
void CheatIO::initRecv(){
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &CheatIO::StateCallback, this);
    _servo_sub[0] = _nm.subscribe( "/" + _robot_name + "_gazebo/L_hip_controller/state", 1, &CheatIO::LhipCallback, this);
    _servo_sub[1] = _nm.subscribe( "/" + _robot_name + "_gazebo/L_hip2_controller/state", 1, &CheatIO::Lhip2Callback, this);
    _servo_sub[2] = _nm.subscribe( "/" + _robot_name + "_gazebo/L_thigh_controller/state", 1, &CheatIO::LthighCallback, this);
    _servo_sub[3] = _nm.subscribe( "/" + _robot_name + "_gazebo/L_calf_controller/state", 1, &CheatIO::LcalfCallback, this);
    _servo_sub[4] = _nm.subscribe( "/" + _robot_name + "_gazebo/L_toe_controller/state", 1, &CheatIO::LtoeCallback, this);
    _servo_sub[5] = _nm.subscribe( "/" + _robot_name + "_gazebo/R_hip_controller/state", 1, &CheatIO::RhipCallback, this);
    _servo_sub[6] = _nm.subscribe( "/" + _robot_name + "_gazebo/R_hip2_controller/state", 1, &CheatIO::Rhip2Callback, this);
    _servo_sub[7] = _nm.subscribe( "/" + _robot_name + "_gazebo/R_thigh_controller/state", 1, &CheatIO::RthighCallback, this);
    _servo_sub[8] = _nm.subscribe( "/" + _robot_name + "_gazebo/R_calf_controller/state", 1, &CheatIO::RcalfCallback, this);
    _servo_sub[9] = _nm.subscribe( "/" + _robot_name + "_gazebo/R_toe_controller/state", 1, &CheatIO::RtoeCallback, this);
}

void CheatIO::StateCallback(const gazebo_msgs::ModelStates & msg)
{
    int robot_index;
    // std::cout << msg.name.size() << std::endl;
    for(int i = 0; i < msg.name.size(); i++)
    {
        if(msg.name[i] == _robot_name + "_gazebo")
        {
            robot_index = i;
        }
    }

    _highState.position[0] = msg.pose[robot_index].position.x;
    _highState.position[1] = msg.pose[robot_index].position.y;
    _highState.position[2] = msg.pose[robot_index].position.z;

    _highState.velocity[0] = msg.twist[robot_index].linear.x;
    _highState.velocity[1] = msg.twist[robot_index].linear.y;
    _highState.velocity[2] = msg.twist[robot_index].linear.z;

    _highState.imu.quaternion[0] = msg.pose[robot_index].orientation.w;
    _highState.imu.quaternion[1] = msg.pose[robot_index].orientation.x;
    _highState.imu.quaternion[2] = msg.pose[robot_index].orientation.y;
    _highState.imu.quaternion[3] = msg.pose[robot_index].orientation.z;

    _highState.imu.gyroscope[0] = msg.twist[robot_index].angular.x;
    _highState.imu.gyroscope[1] = msg.twist[robot_index].angular.y;
    _highState.imu.gyroscope[2] = msg.twist[robot_index].angular.z;
}

/**
 * @brief 左髋关节电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::LhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[0].mode = msg.mode;
    _highState.motorState[0].q = msg.q;
    _highState.motorState[0].dq = msg.dq;
    _highState.motorState[0].tauEst = msg.tauEst; //力矩估计值
}

/**
 * @brief 左髋关节2电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::Lhip2Callback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[1].mode = msg.mode;
    _highState.motorState[1].q = msg.q;
    _highState.motorState[1].dq = msg.dq;
    _highState.motorState[1].tauEst = msg.tauEst;
}

/**
 * @brief 左大腿电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::LthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[2].mode = msg.mode;
    _highState.motorState[2].q = msg.q;
    _highState.motorState[2].dq = msg.dq;
    _highState.motorState[2].tauEst = msg.tauEst;
}

/**
 * @brief 左小腿电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::LcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[3].mode = msg.mode;
    _highState.motorState[3].q = msg.q;
    _highState.motorState[3].dq = msg.dq;
    _highState.motorState[3].tauEst = msg.tauEst;
}

/**
 * @brief 左脚板电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::LtoeCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[4].mode = msg.mode;
    _highState.motorState[4].q = msg.q;
    _highState.motorState[4].dq = msg.dq;
    _highState.motorState[4].tauEst = msg.tauEst;
}

/**
 * @brief 右髋关节电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::RhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[5].mode = msg.mode;
    _highState.motorState[5].q = msg.q;
    _highState.motorState[5].dq = msg.dq;
    _highState.motorState[5].tauEst = msg.tauEst;
}

/**
 * @brief 右髋关节2电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::Rhip2Callback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[6].mode = msg.mode;
    _highState.motorState[6].q = msg.q;
    _highState.motorState[6].dq = msg.dq;
    _highState.motorState[6].tauEst = msg.tauEst;
}

/**
 * @brief 右大腿电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::RthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[7].mode = msg.mode;
    _highState.motorState[7].q = msg.q;
    _highState.motorState[7].dq = msg.dq;
    _highState.motorState[7].tauEst = msg.tauEst;
}

/**
 * @brief 右小腿电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::RcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[8].mode = msg.mode;
    _highState.motorState[8].q = msg.q;
    _highState.motorState[8].dq = msg.dq;
    _highState.motorState[8].tauEst = msg.tauEst;
}

/**
 * @brief 右脚板电机反馈
 * @details 调用宇树电机的类，将宇树电机的数据接入
 * @param msg 宇树腿部信息 的 电机状态 的 信息变量 message
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void CheatIO::RtoeCallback(const unitree_legged_msgs::MotorState& msg)
{
    _highState.motorState[9].mode = msg.mode;
    _highState.motorState[9].q = msg.q;
    _highState.motorState[9].dq = msg.dq;
    _highState.motorState[9].tauEst = msg.tauEst;
}