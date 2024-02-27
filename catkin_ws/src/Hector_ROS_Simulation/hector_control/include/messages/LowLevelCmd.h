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

#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "../common/cppTypes.h"
// 宇树科技的关节电机包含如下 5 个控制指令：
// 1. 前馈力矩：τff 单位：Nm, |T|<128
// 2. 期望角度位置：pdes 单位：rad, |Pos|<823549
// 3. 期望角速度：ωdes 单位：rad/s, |W|<256
// 4. 位置刚度：kp 0<K_P<16
// 5. 速度刚度（阻尼）：kd  0<K_W<32
// 6. 9.1 为减速比。
// 在关节电机的混合控制中，使用 PD 控制器将电机在输出位置的偏差反馈到力矩输出上：
// τ = τf f + kp · (pdes − p) + kd · (ωdes − ω)
// τ 为关节电机的电机转子输出力矩，p 为电机转子的当前角度位置，ω 为
// 电机转子的角速度。在实际使用关节电机时，需要注意将电机输出端的控制目标量与发
// 送的电机转子的指令进行换算。
// mode:目标电机运行模式。0 为停转，5 为开环缓慢转动，10 为闭环伺服控制
struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;
    
    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0; //关节电机的电机转子输出力矩
        Kp = 0; //位置刚度
        Kd = 0; //速度刚度（阻尼）
    }
};

/**
 * @brief 底层的命令码，即电机各状态
 * @details 将两条腿十个电机的电机命令码包装
 * @author wyt
 * @date 2023.11.20
 */
struct LowlevelCmd{
    MotorCmd motorCmd[10];
};

#endif // LOWLEVELCMD_H