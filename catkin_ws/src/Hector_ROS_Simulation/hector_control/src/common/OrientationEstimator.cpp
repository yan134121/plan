#include "../../include/common/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 * 从矢量导航IMU获得 四元数，旋转矩阵，角速度(身体的和世界的)，rpy，加速度(世界的，身体的)
 */
// 由于是从CheatIO中得到的方向传感器数据，所以命名为Cheat方向估计器
void CheaterOrientationEstimator::run() {
  //std::cout << "orientation" << std::endl;
  // 接收四元数数据
  this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.lowState->imu.quaternion[0];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.lowState->imu.quaternion[1];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.lowState->imu.quaternion[2];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.lowState->imu.quaternion[3];
    // 由四元数得到的3x3旋转矩阵,表示 世界坐标系在机器人身体坐标系下的变换,身体坐标系下发生的旋转要转到世界坐标系下需要左乘rBody转置
  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);
    // 接收陀螺仪三轴rpy信息
  this->_stateEstimatorData.result->omegaBody(0) =
      this->_stateEstimatorData.lowState->imu.gyroscope[0];
  this->_stateEstimatorData.result->omegaBody(1) =
      this->_stateEstimatorData.lowState->imu.gyroscope[1];
  this->_stateEstimatorData.result->omegaBody(2) =
      this->_stateEstimatorData.lowState->imu.gyroscope[2];
      this->_stateEstimatorData.result->rpy =
    ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    // 将机器人自身坐标下的姿态转到世界坐标系下的姿态 没有位移
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);
      
}
