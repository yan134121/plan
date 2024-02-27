#include "../../include/common/PositionVelocityEstimator.h"
// 由于是从CheatIO中得到的位移和速度传感器数据，所以命名为Cheat位置速度估计器
void CheaterPositionVelocityEstimator::run() {
 // std::cout << "run StateEstimator" << std::endl;
  for(int i = 0; i < 3; i++){
    this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
  }
  // 获得身体坐标系下的速度
  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
}
