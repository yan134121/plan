/*
BSD 3-Clause License

Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LOWLEVELSTATE_H
#define LOWLEVELSTATE_H

#include <iostream>
#include <string>
#include "../common/cppTypes.h"
#include "../common/enumClass.h"

struct UserValue{
    //键盘按键
    float lx;//D->+   A->-   在y轴方向移动
    float ly;//W->+    S->-  在x轴方向移动，按一下速度增减0.025*1.5
    float rx;//L->+   J->-从上往下看，逆时针旋转 按一下旋转速率增减0.025*2
    float ry;//I->+   k->-从上往下看，顺时针旋转，有定义，但是实际上算法中没有使用
    float L2; // 没用上
    float vx; // vx in body frame 没用上
    float vy; // vy in body frame 没用上
    float turn_rate; // 没用上
    // 构造函数 初始化调用归零函数，全为零
    UserValue(){
        setZero();
    }
    // 归零函数
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
        vx = 0;
        vy = 0;
        turn_rate = 0;
    }
};

struct MotorState
{
    unsigned int mode;
    float q; // 位置
    float dq; // 速度
    float ddq; // 加速度
    float tauEst; // 力矩估计
    // 初始化赋值
    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4]; // 四元数
    float gyroscope[3]; // 陀螺仪
    float accelerometer[3]; // 加速度计
    // 构造函数 初始化调用归零函数，全为零
    IMU()
    {
        for(int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }
};

/**
 * @brief 底层状态
 * @details 包括陀螺仪、电机状态、用户命令码、用户数值
 *          xyz三轴位置，三轴世界速度、三轴角度
 * @author wyt
 * @date 2023.11.20
 */
struct LowlevelState
{
    IMU imu;
    MotorState motorState[10];
    UserCommand userCmd;
    UserValue userValue;

    float position[3];
    float vWorld[3];
    float rpy[3];
    // 初始化为零
    LowlevelState()
    {
        for(int i = 0; i < 3; i++)
        {
            position[i] = 0; //位置
            vWorld[i] = 0; //速度
            rpy[i] = 0; //角度
        }
    }
};


#endif //LowlevelState