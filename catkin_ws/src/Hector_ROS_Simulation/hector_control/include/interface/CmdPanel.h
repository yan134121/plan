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

#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "../messages/unitree_joystick.h"
#include "../common/enumClass.h"
#include "../sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "../messages/LowlevelState.h"
#include <pthread.h>

struct WaypointCmd{
    float x;
    float y;
    float yaw;
    int mode;
    WaypointCmd(){
        setZero();
    }
    void setZero(){
        x = 0;
        y = 0; 
        yaw = 0;
        mode = 0;
    }
};

/**
 * @brief ，命令面板 类
 * @details 可以获取用户命令,用户数值,
 *          可以设置为被动模式,用户命令码直接赋值为L2_B
 *          可以设置为零输出模式,全部置零
 *          虚拟函数用于在派生类中实现快速扩充，但未实现
 * @author wyt
 * @date 2023.11.20
 */
class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
    //通过函数调用私有成员 userCmd，保护数据
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    // 强制将用户命令置为L2_B 被动模式,不能接收到运动控制命令数据
    void setPassive(){userCmd = UserCommand::L2_B;}
    void setZero(){userValue.setZero();}
    // 该虚拟函数 接收处理回调函数
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
    // 该虚拟函数 更新速度命令
    virtual void updateVelCmd(const LowlevelState *state){};

protected:
    // 虚拟函数
    virtual void *run(void *arg){};
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H