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

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Biped.h"


/*!
 * Data sent from control algorithm to legs
 */ 
    struct LegControllerCommand{
        LegControllerCommand() {zero();}
        // 数据归零函数
        void zero();
        //Des期望值 希腊字母τ 力矩
        Vec5<double> qDes, qdDes, tau; //希腊字母τ 力矩
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        // 前馈力  前馈控制：控制系统中采取预先计算或预测的控制策略，将其直接应用于系统以实现期望的系统响应，
        // 目的：在系统受到外部扰动或变化时提供对应的控制输入，以维持系统的性能和稳定性
        Vec6<double> feedforwardForce;  
        Vec3<double> hiptoeforce; //hip->toe的力?
        Mat3<double> kpCartesian; //足端的力pd控制的kp笛卡尔值
        Mat3<double> kdCartesian; //足端的力pd控制的kd笛卡尔值
        double kptoe; //脚趾的kp
        double kdtoe; //脚趾的kd
    };

/*!
 * Data returned from legs to control code
 */ 
    struct LegControllerData{
        //结构体的构造函数
        LegControllerData() {zero();}
        void setBiped(Biped& biped) { hector = &biped; }

        void zero();
        // 5x1矢量
        Vec5<double> q, qd; // 五个电机的位置 速度
        Vec3<double> p, v; // 足端位置和足端速度 以hip1为原点的坐标系下
        Mat65<double> J_force_moment; // 6x5 关节joint力矩
        Mat35<double> J_force; // 3x5 关节joint力
        Vec5<double> tau; //τ
        Biped* hector; // 没用上
    };

/*!
 * Controller for 2 legs of hector
 */ 
/**
 * @brief 腿控制器
 * @details  
 * @author wyt
 * @date 2023.11.20
 */
    class LegController {
      public:
      //Eigen 库中的一个宏 确保内存对齐
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //构造函数，只初始化第一次，biped类中的信息作为参数传入
        LegController(Biped& biped) : _biped(biped) {
            // 范围循环（range-based for loop）语法。这行代码的作用是遍历 data 数组中的
            // 每个元素（dat 是每次迭代中当前元素的别名），然后调用 setBiped 函数并传递 _biped 对象作为参数。
            // data是LegControllerData结构体类型，而且两条腿所以是两个，将_biped分别给到两条腿
            for (auto& dat : data) dat.setBiped(_biped);
            for(int i = 0; i < 2; i++){
                // 清空下发电机力矩控制命令和上传电机反馈数据
                commands[i].zero();
                data[i].zero();
            }
        };
        // 清空下发命令函数
        void zeroCommand();
        // 没有实现函数
        void edampCommand(double gain);
        // 更新上传数据函数
        void updateData(const LowlevelState* state);
        // 更新下发命令
        void updateCommand(LowlevelCmd* cmd);
        // 虚函数，初始化一个值，可被替代
        void setEnabled(bool enabled) {_legsEnabled = enabled;};

        LegControllerCommand commands[2];
        LegControllerData data[2];  //两个应该是两条腿，但该结构体里面的Biped类是整体的
        bool _legsEnabled = false;
        // 髋关节1 髋关节2 大腿 膝关节 脚趾
        std::string limbName[5] = {"Hip 1", "Hip 2", "Thigh", "Knee ", "Toe  "}; 
        std::string Side[2] = {"Left ", "Right"};        
        Biped& _biped;
    };

    void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg);

#endif