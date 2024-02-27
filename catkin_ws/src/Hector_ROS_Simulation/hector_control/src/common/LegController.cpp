#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>

// upper level of joint controller 
// send data to joint controller

/**
 * @brief 腿控制器命令中的归零函数
 * @details 将所有的数据都归零，应该是上层传下来的数据
 * @param 
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void LegControllerCommand::zero(){
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
    double kptoe = 0;
    double kdtoe = 0;
}

/**
 * @brief 腿控制器数据中的归零函数
 * @details 将所有的数据都归零，是底层收集的数据
 * @param 
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

/**
 * @brief 腿控制器数据中的归零函数
 * @details 将所有的数据都归零，是底层收集的数据
 * @param 
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void LegController::zeroCommand(){
    for (int i = 0; i < 2; i++){
        commands[i].zero();
    }
}

/**
 * @brief 腿控制器中的更新数据函数
 * @details 收集电机的位置 速度 力矩信息
 * @param LowlevelState* state 底层状态 包括陀螺仪、电机状态、用户命令码、用户数值 xyz三轴位置，三轴世界速度、三轴角度
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void LegController::updateData(const LowlevelState* state){
    for (int leg = 0; leg < 2; leg++)
    {    //将电机的状态信息传给腿控制器，
        for(int j = 0; j < 5; j++){
            data[leg].q(j) = state->motorState[leg*5+j].q;       //        LegControllerData data[2];
            data[leg].qd(j) = state->motorState[leg*5+j].dq;     
            data[leg].tau(j) = state->motorState[leg*5+j].tauEst;
            std::cout << "motor joint data" << leg*5+j << ": "<< data[leg].q(j) << std::endl;
        }
        // 计算腿的雅可比矩阵和位置
        computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        // 文章Force-and-moment-based Model Predictive Control 
        // for Achieving Highly Dynamic Locomotion on Bipedal Robots
        // 公式28. 由于速度雅各比的转置就是力的雅各比所以使用J_force
        //data[leg].qd 各关节速度、data[leg].J_force 关节力的雅可比
        // 脚底与地面等效接触点速度 踝关节正下方 在身体坐标系
        // 用于在控制电机中做pd控制的速度控制
        data[leg].v = data[leg].J_force * data[leg].qd;   
    }
}

/**
 * @brief 腿控制器中的更新命令函数
 * @details 将MPC支撑力和PD摆动力通过雅可比分配到每个电机
 * @param LowlevelCmd* cmd 十个电机的命令码
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void LegController::updateCommand(LowlevelCmd* cmd){
    // 两条腿
    for (int i = 0; i < 2; i++)
    {
        /*"Feedforward force"（前馈力）是指系统中对控制系统施加的预定的、
           基于先验知识或期望行为的力。这是一种在控制系统中引入的先验信息，
           以提高系统性能或实现特定的期望输出
           在该系统中由 qp二次规划 求解得出的 MPC 预测10步中的最近一步中两只脚所需三轴力和三轴力矩 
        */
        Vec6<double> footForce = commands[i].feedforwardForce;  
        // 腿部力矩五个电机 = 腿部力矩的转置 * 脚力和力矩，这个脚力和力矩是前馈力，是由地面反作用力得到，而地面反作用力是由qp计算得到
        Vec5<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg从站立腿开始用力
        // 五个电机
        for(int j = 0; j < 5; j++){
            std::cout << "legtau" << j << ": "<< legtau(j) << std::endl;
        }

        // cartesian PD control for swing foot 摆动脚的直角PD控制
        if(commands[i].kpCartesian(0,0) != 0 || commands[i].kdCartesian(0,0) != 0)
        {
            //足端的力，PD控制，公式（30）
            Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                        commands[i].kdCartesian * (commands[i].vDes - data[i].v);
                
            //腿上5个电机的转矩，公式（31）
            Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d ; 

            // maintain hip angle tracking 保持髋部角度跟踪
            double kphip1 = 15;
            double kdhip1 = 1;
            // 重新赋值髋部的摆动力矩 ！！！！！！！！！！！！注意 这里是认为hip1的电机位置在上电时就是0，如果不改这里，需要在获取电机位置的地方减去偏移
            swingtau(0) = kphip1*(0-data[i].q(0)) + kdhip1*(0-data[i].qd(0));
            // make sure foot is parallel with the ground 确保脚与地面平行 脚趾的摆动力矩 重新赋值
            swingtau(4) = commands[i].kptoe * (-data[i].q(3)-data[i].q(2)-data[i].q(4))+commands[i].kdtoe*(0-data[i].qd(4));

            // 实际上只有髋部和脚的力矩有变化
            for(int j = 0; j < 5; j++)
            {
                legtau(j) += swingtau(j);
            }
        }
        // 再把每条腿的力矩矩阵合并起来
        commands[i].tau += legtau;
        // 将上层的控制数据赋值给底层的电机
        for (int j = 0; j < 5; j++)
        {
            // 出来tau力矩外其他没有赋值，相当于在底层电机是开环控制，闭环本算法里面
            cmd->motorCmd[i*5+j].tau = commands[i].tau(j);
            cmd->motorCmd[i*5+j].q = commands[i].qDes(j);       
            cmd->motorCmd[i*5+j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i*5+j].Kp = commands[i].kpJoint(j,j);
            cmd->motorCmd[i*5+j].Kd = commands[i].kdJoint(j,j);
            std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " << cmd->motorCmd[i*5+j].tau << std::endl;            
        }
        // 传递完毕后将上层的力矩恢复成零
        commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference 零扭矩指令，防止干扰
        
    }    
    //std::cout << "cmd sent" << std::endl;
}

/**
 * @brief 计算腿的雅可比矩阵和位置
 * @details 
 * @param _biped 机器人的一些基本参数信息，没用上
 *        q 单条腿五个电机的角度
 * 
 *        J_f 力的雅各比矩阵
 *        p 脚底与地面等效接触点 踝关节正下方 在身体坐标系
 *        leg 选择哪条腿
 * @return void
 * @author wyt
 * @date 2024.01.27
 */
void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg)
{
    //此函数涉及到关节逆运动学解算，主要是求解腿部5个关节和控制变量的雅可比矩阵。

    // 从直膝转到屈膝 以屈膝为初始位置
    q(2) = q(2) + 0.3*3.14159;     //此电机正向转180°*30%=54°？
    q(3) = q(3) - 0.6*3.14159;     //逆向转108°？
    q(4) = q(4) + 0.3*3.14159;     //正向转54°？

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    
    double side = -1.0; // 1 for Left legs; -1 for right legs
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    // std::cout<< "Leg Sign" << side << std::endl;
    
    
    // 检查指针是否为非空 因为传入的是指针需要保证安全
    if(J_f_m)
    {
    J_f_m->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f_m->operator()(2, 0) =  0.0;
    J_f_m->operator()(3, 0) = 0.0;
    J_f_m->operator()(4, 0) = 0.0;
    J_f_m->operator()(5, 0) = 1.0;

    J_f_m->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);
    J_f_m->operator()(3, 1) = cos(q0);
    J_f_m->operator()(4, 1) = sin(q0);
    J_f_m->operator()(5, 1) = 0.0;

    J_f_m->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f_m->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(3, 2) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 2) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 2) = sin(q1);

    J_f_m->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f_m->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(3, 3) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 3) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 3) = sin(q1);

    J_f_m->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f_m->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    J_f_m->operator()(3, 4) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 4) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 4) = sin(q1);
   }
//这部分和下面的求力的部分由重合
//这部分叫力和力矩的雅可比矩阵，前面3行是力的雅可比，后面是力矩的雅可比
   if(J_f){
    J_f->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f->operator()(2, 0) =  0.0;

    J_f->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);

    J_f->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));

    J_f->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));

    J_f->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    
    }
   if(p){
    p->operator()(0) = - (3*cos(q0))/200 - (9*sin(q4)*(cos(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))))/250 - (11*cos(q0)*sin(q2))/50 - ( (side)*sin(q0))/50 - (11*cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))/50 - (11*sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2))))/250 - (23*cos(q1)* (side)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50;
    p->operator()(1) = (cos(q0)* (side))/50 - (9*sin(q4)*(cos(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1))))/250 - (3*sin(q0))/200 - (11*sin(q0)*sin(q2))/50 - (11*cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)))/50 - (11*sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2))))/250 + (23*cos(q0)*cos(q1)* (side))/1000 + (11*cos(q0)*cos(q2)*sin(q1))/50;
    p->operator()(2) = (23*(side)*sin(q1))/1000 - (11*cos(q1)*cos(q2))/50 - (9*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/250 + (9*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/250 - (11*cos(q1)*cos(q2)*cos(q3))/50 + (11*cos(q1)*sin(q2)*sin(q3))/50 - 3.0/50.0;
   
   }
}


