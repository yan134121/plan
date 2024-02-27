/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */


#ifndef PROJECT_STATE_ESTIMATOR_CONTAINER_H
#define PROJECT_STATE_ESTIMATOR_CONTAINER_H

#pragma once

#include "LegController.h"
#include "../messages/LowlevelState.h"

/*!
 * Result of state estimation
   状态估计结果
 */

struct StateEstimate {
    //4x1 Vector 接触估计
    Vec4<double> contactEstimate;
    //位置 3x1 Vector
    Vec3<double> position;
    //速度 3x1 Vector
    Vec3<double> vBody;
    //4x1 Vector方位 4x1 Vector 四元数
    Quat<double> orientation; 
    //Ω_Body 身体的角度 3x1 Vector
    Vec3<double> omegaBody;
    // Rotation Matrix 旋转矩阵 3x3
    RotMat<double> rBody; // "跟随世界坐标系"在身体坐标系下的旋转矩阵   由四元数得到quaternionToRotationMatrix
    //欧拉角 roll pitch yaw  3x1 Vector
    Vec3<double> rpy;
    //Ω_World  3x1 Vector
    Vec3<double> omegaWorld;
    //世界速度 3x1 Vector
    Vec3<double> vWorld;
    //身体加速度 世界加速度  3x1 Vector
    Vec3<double> aBody, aWorld;
};


/*!
 * input for state estimation
 */ 
struct StateEstimatorData {
    StateEstimate* result;
    LowlevelState* lowState;
    LegControllerData* legControllerData;
};

/*!
 * All Estimators inherit from this class
 */ 
class GenericEstimator{
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData data) {_stateEstimatorData = data;};

    virtual ~GenericEstimator() = default;
    StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator class 主状态估计器类
 * Contains all GenericEstimator包含所有泛型估计器
 */ 
class StateEstimatorContainer {
  public:
    // Constructor 
    //Eigen 库中的一个宏 确保内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //构造函数
    StateEstimatorContainer(LowlevelState *_lowState,
                            LegControllerData *_legControllerData,
                            StateEstimate *stateEstimate){
        // 一个结构体，包含了与状态估计相关的数据，如低级状态、腿控制器数据和状态估计结果
        _data.lowState = _lowState;
        _data.legControllerData = _legControllerData;
        _data.result = stateEstimate;
    }
    // deconstructor 解构函数
    ~StateEstimatorContainer() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
    }
    // run estimator 运行估计量
    void run(){
        for (auto estimator : _estimators){
            estimator->run();
        }
        //每循环一次将_estimators的一个元素复制给estimator，然后run()
        //这里的_estimators是GenericEstimator的动态数组liu
    }

    // get result  获取 所有姿态估计 的结果 
    const StateEstimate&  getResult() {return *_data.result;}

    // add estimator of given type 添加给定类型的估计器
    template <typename EstimatorToAdd>
    void addEstimator(){
        std::cout << "add estimator" << std::endl;
        auto* estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        // 在容器的末尾添加元素的函数
        _estimators.push_back(estimator);
    }

    // remove estimator of given type 移除给定类型的估计器
    template <typename EstimatorToRemove>
    void removeEstimator() {
        int nRemoved = 0;
        _estimators.erase(
            std::remove_if(_estimators.begin(), _estimators.end(),
                           [&nRemoved](GenericEstimator* e){
                               if (dynamic_cast<EstimatorToRemove*>(e)){
                                   delete e;
                                   nRemoved++;
                                   return true;
                                } else {
                                    return false;
                                }
                           }),
            _estimators.end());
    }

    // remove all estimators 移除所有的估计器
    void removeAllEstimators() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
        _estimators.clear();
    }
  private:
    std::vector<GenericEstimator*> _estimators;
    Vec4<double> _phase;
    StateEstimatorData _data;
};


#endif