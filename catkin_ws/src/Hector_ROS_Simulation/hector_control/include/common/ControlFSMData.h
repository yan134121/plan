#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "LegController.h"
#include "Biped.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "../interface/IOInterface.h"
#include "StateEstimatorContainer.h"

/**
 * @brief 控制有限状态机的数据
 * @details 把所有和控制有关的类都包含进来
 * @author wyt
 * @date 2023.11.20
 */
struct ControlFSMData {
  //Eigen 库中的一个宏 确保内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //创建一个两足(Biped)的类
  Biped *_biped;
  StateEstimatorContainer *_stateEstimator; // 状态估计器容器
  LegController *_legController; // 腿部控制器
  DesiredStateCommand *_desiredStateCommand; // 期望状态命令
  IOInterface *_interface; // 调用宇树的人机交互函数
  LowlevelCmd *_lowCmd; // 包装底层电机参数的结构体
  LowlevelState *_lowState; // 电机的反馈参数 传感器的数据等等
  // 发送接收电机和传感器数据的通信函数
  void sendRecv(){ 
    // 在 IOInterface *_interface 中有虚函数 sendRecv 
    // 而 CheatIO 继承了 IOInterface 并实现了函数sendRecv，
    // 所以反过来父类调用子类的函数
    _interface->sendRecv(_lowCmd, _lowState);
  }
};


#endif  // CONTROLFSM_H