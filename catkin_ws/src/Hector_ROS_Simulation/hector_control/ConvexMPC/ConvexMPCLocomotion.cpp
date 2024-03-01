#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;
using Eigen::Dynamic;


/* =========================== Controller ============================= */

/**
 * @brief 凸MPC运动 传参从FSMState_Walking的Cmpc(0.001, 40)进来
 * @details 
 * @param _dt 0.001
 *        _iterations_between_mpc MPC之间的迭代 40
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : 
 iterationsBetweenMPC(_iterations_between_mpc),
 horizonLength(10),
 dt(_dt),
  // 初始化7个步态类  Gait 将这些参数写入Gait的构造函数中
 // 第三个参数durations持续时间 速度越快,双腿支撑MPC的时间越短, 所以站立的时候是100% 10,跳跃是蓄力一段时间然后迅速一跃,所以是90% 9
 galloping(horizonLength, Vec2<int>(0, 2), Vec2<int>(5, 5), "Galloping"), // 疾驰
 pronking(horizonLength, Vec2<int>(0, 0), Vec2<int>(4, 4), "Pronking"),   // 俯身
 trotting(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Trotting"),   // 小跑
 bounding(horizonLength, Vec2<int>(0, 0), Vec2<int>(9, 9), "Bounding"),   // 跳跃
 walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),     // 行走
 pacing(horizonLength, Vec2<int>(0, 9), Vec2<int>(5, 5), "Pacing"),       // 踱步
 standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")  // 站立
{
  // 7种步态
  gaitNumber = 7;  
  // 0.001 * 40
  dtMPC = dt * iterationsBetweenMPC; 
  // yaw轴强制为0
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    // 第一次摆动标志位  置1
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");
}

/******************************************************************************************************/
/******************************************************************************************************/

/**
 * @brief 凸MPC运动运行函数
 * @details 
 * @param _dt 
 *        _iterations_between_mpc MPC之间的迭代
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;
  // C++11 引入的 auto 关键字可以自动推导变量的类型
  auto &seResult = data._stateEstimator->getResult();//从状态估计器获得机器人的状态信息，contactestimate, position, vbody, orientation,等
  auto &stateCommand = data._desiredStateCommand;//获得机器人的遥控器指令。
  // 选择要装载的行走模式
  // pick gait 步态 快步，小跑
  Gait *gait = &trotting;
  if (gaitNumber == 1)
    gait = &bounding; // 跳跃
  else if (gaitNumber == 2)
    gait = &trotting; // 快步，小跑
  else if (gaitNumber == 3)
    gait = &walking; // 走
  else if (gaitNumber == 4)
    gait = &pacing; // 踱步
  else if (gaitNumber == 5)
    gait = &galloping; //急速进行
  else if (gaitNumber == 6)
    gait = &pronking; //时跑时跳
  else if (gaitNumber == 7)
    gait = &standing; // 站立
    // 当前步态
  current_gait = gaitNumber;

  // integrate position setpoint 积分位置设定值  
  // 机器人质心的目标速度包括x轴、y轴、z轴为0. 由遥控器输入
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);

  Vec3<double> v_des_world;

  // rBody只是一个3x3的姿态变换矩阵，因此只是将机器人自身在imu坐标下遥控器速度转到平行于真实世界坐标，且原点与imu坐标重合跟随机器人移动的“跟随世界坐标”
  v_des_world = seResult.rBody.transpose() * v_des_robot; // 机器人自身期望速度转到“跟随世界坐标”下的期望速度

  Vec3<double> v_robot = seResult.vWorld;    //机器人的速度 = 世界坐标系速度
  // 除去 firstRun上电第一次运行，期待的世界xy位置由速度积分得到，而高度z轴强制为0.55
  world_position_desired[0] += dt * v_des_world[0];    //机器人世界坐标系下的位置=位置+dt*速度，x轴        目标位置
  world_position_desired[1] += dt * v_des_world[1];     //y轴                                           目标位置
  world_position_desired[2] = 0.55; //.5;;;           //z轴设置为固定值。

  // get then foot location in world frame 在世界框架中得到脚的位置
  for (int i = 0; i < 2; i++)
  {
    // 起脚点
    //每只脚的位置=机器人当前位置+旋转矩阵*（偏移量hip的偏移量（和结构相关）+足端的位置（这是以hip为坐标原点求的））。
    //公式（29）         这篇文章：Force-and-moment-based Model Predictive Control for Achieving Highly Dynamic
    // data._legController->data[i].p由雅各比计算得到
    pFoot[i] = seResult.position + seResult.rBody.transpose()*(data._biped->getHip2Location(i) + data._legController->data[i].p);
  }

  // some first time initialization 一些首次初始化

  if (firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    // 上电后第一次强制覆盖将期待的世界坐标 = 状态估计器所获取的估计位置，应该是(0,0,0)
    world_position_desired[0] = seResult.position[0]; // 期待的世界坐标x轴位置
    world_position_desired[1] = seResult.position[1]; // 期待的世界坐标y轴位置
    world_position_desired[2] = seResult.position[2]; // 期待的世界坐标z轴位置

    // connect to desired state command later 稍后连接到所需的状态命令
    Vec3<double> v_des_robot(0, 0, 0); // 期待的机器人坐标系速度
    Vec3<double> v_des_world(0, 0, 0); // 期待的世界坐标系速度

    Vec3<double> v_robot = seResult.vWorld;
    // 期待的机器人位置 = 期待的世界坐标系位置。刚上电 身体位置与世界坐标位置重合
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    // 期待的机器人速度 = 期待的世界坐标系速度
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0; // z轴位置要保持，所以期待速度为0
    // 期待的机器人欧拉角度
    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];
    // 期待的机器人朝向orientation，应该是是四元数，但是只有三个元素
    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    // 判断上电时的运动模式 即站立模式 standing
    if (gaitNumber == 7)
    {
      // 覆盖 期待的机器人位置 由 状态估计器获得 高度z轴强制为0.55
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = 0.55;

      vBody_des[0] = 0;
      vBody_des[1] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      // 设置足部摆动轨迹的最高值0.05m
      footSwingTrajectories[i].setHeight(0.05);
      // 设置上电时轨迹的起点 就是 站立时的位置
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      // 设置上电时轨迹的起点 就是 站立时的位置
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }

//起点和终点为什么都是pFoot[i]


    // 上电初始化完毕 一次就行
    firstRun = false;
  }
  // 以上初始化完毕
  // 获取两只脚的接触状态，返回当前相位（2x1列向量） 没用上
  contact_state = gait->getContactSubPhase();

  // foot placement 脚位置
  //悬空的时间=MPC计算周期的时间步长*腿悬空是步数 dtMPC = dt * iterationsBetweenMPC=0.04  
  // 以行走模式为例  swingTimes[0]=0.04*5=0.2   _swing=10-5=5; _swing = nMPC_segments - durations[0]
  swingTimes[0] = dtMPC * gait->_swing;  
  swingTimes[1] = dtMPC * gait->_swing;
  //side_sign 是一个标志位，用于确定腿部的摆动方向，即腿部向前摆动（1）或向后摆动（-1）。
  // 这样可以实现腿部的协同摆动，使机器人能够保持平衡。
  double side_sign[2] = {1, -1};
                                
  // y轴的交错，用于控制腿部摆动时的水平偏移。通过调整这些值，
  // 可以使机器人的腿部在摆动时产生一些横向的运动，从而实现更自然的步行。
  double interleave_y[2] = {-0.1, 0.1};
  
  //它表示横向运动的增益，用于调整腿部的横向位移。这可以影响机器人的步行稳定性和速度。
  double interleave_gain = -0.2;
  
  // 线速度绝对值 通常用于步态控制中的一些调整和决策。
  double v_abs = std::fabs(seResult.vBody[0]);

  // 两条腿
  for (int i = 0; i < 2; i++)
  {
    // 判断是否第一次摆动
    if (firstSwing[i])
    {
      // 上电后第一次摆动时，剩余摆动时间 = 摆动时间
      //  以行走模式为例  _swing=0.04*5=0.2
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    { 
      // 其他时候的每次减去dt 从这里可以看出，该程序更新的周期为dt
      swingTimeRemaining[i] -= dt; // dt=0.001 0.2/0.001=200次
    }

    // if (firstSwing[i])
    // {
      // 更新脚的摆动轨迹
      footSwingTrajectories[i].setHeight(0.1);  //腿部摆动的最高高度为0.1m

      Vec3<double> offset(0, side_sign[i] * 0.0, -0.0);      //补偿都是0，调节该系数可以控制步态 

      // simple heuristic function 简单启发式函数
      //通过将机器人髋关节位置与偏移量相加来计算的。这个偏移量通常用于调整腿部的摆动轨迹，以实现一些特定的运动要求或步态控制策略。
      //这个变量只是一个偏移量参数。
      Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);   
      
      Vec3<double> des_vel;

      des_vel[0] = stateCommand->data.stateDes(6); // 获取遥控器命令的x轴坐标速度 此坐标速度是在机器人自身坐标下，即以x轴为前进后退方向，y轴为左右方向
      des_vel[1] = stateCommand->data.stateDes(7); // 获取遥控器命令的y轴坐标速度
      des_vel[2] = stateCommand->data.stateDes(8); // 获取遥控器命令的z轴坐标速度

      /*
        Pf 表示腿部摆动轨迹的期望终点位置，通常用于计算腿部摆动轨迹的目标位置。
        seResult.position 是机器人的当前位置，通常是机器人的质心位置（center of mass）。它表示了机器人的当前全局坐标系中的位置。
        seResult.rBody.transpose() * pRobotFrame 表示了机器人本体坐标系中的腿部摆动轨迹的期望位置。pRobotFrame 是之前计算的机器人本体坐标系下的位置偏移。
        seResult.vWorld 表示机器人的全局坐标系中的线速度。它表示了机器人的速度和方向。
        swingTimeRemaining[i] 表示当前腿部摆动的剩余时间。这个时间通常随着时间的推移递减，表示腿部摆动的进行。
        对比上面的pFoot[i]  起脚点
        pFoot[i] = seResult.position + seResult.rBody.transpose()*(data._biped->getHip2Location(i) + data._legController->data[i].p);
      */
      //  落脚点Pf
      Vec3<double> Pf = seResult.position +                        // 机器人的当前位置 在世界坐标系下
                        seResult.rBody.transpose() * pRobotFrame + // 将髋关节在身体坐标的偏移 转到“跟随世界坐标系”下的偏移
                        seResult.vWorld * swingTimeRemaining[i];   // 机器人三轴运动速度*剩余摆动时间=位移 在世界坐标系下

      // 下面是做一些xy轴的偏移量，加到落脚点的位置上，提高落脚点的拟态或提高落脚点的变化
      // 水平方向上的最大偏移量 限制摆动的幅度
      double p_rel_max = 0.4; 
      // // 以行走模式为例 _stance=durations[0]=5  dtMPC=0.4
      double pfx_rel = -0.015 + seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                       0.02 * (seResult.vWorld[0] - v_des_world[0]);                 // 0.02(当前机器人三轴速度-遥控器命令的三轴速度) 都是在世界坐标系下
                       
                       // 落脚点的偏移量=v*dt            暂时不明白gait->_stance在此处的意思。

      double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                       0.02 * (seResult.vWorld[1] - v_des_world[1]);

      // fmaxf 返回两个浮点数中的最大值 fminf 返回两个浮点数中的最小值 实现限幅
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      // std:: cout << "pfy_rel =" << pfy_rel << "\n";
      // 将添加的动态偏移量加入落脚点中，由于落脚点在平地所以，z轴强制为0
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = -0.0;
      //更新步态规划的落脚点_pf
      footSwingTrajectories[i].setFinalPosition(Pf);  
    // }
  }


  // calc gait  calculation gait 计算步态 设置迭代次数 两次MPC之间的迭代次数iterationsBetweenMPC=40 迭代计数器iterationCounter=10
  gait->setIterations(iterationsBetweenMPC, iterationCounter); 

  // load LCM leg swing gains 加载关于腿部摆动（leg swing）的控制增益（gains）的信息 即pid控制
  Kp << 300,   0,   0,
          0, 300,   0,
          0,   0, 300;
  Kp_stance =  0* Kp;

  Kd << 10,  0,  0,
         0, 10,  0,
         0,  0, 10;
  Kd_stance = 0*Kd;
  // gait
  Vec2<double> contactStates = gait->getContactSubPhase();   //2只脚当前支撑的状态 所处的相位
  Vec2<double> swingStates = gait->getSwingSubPhase();       //2只脚当前悬空的状态 所处的相位
  // 获取MPC步态表
  int *mpcTable = gait->mpc_gait();      

  //每10次程序迭代，运行一次mpc
  updateMPCIfNeeded(mpcTable, data, omniMode);
  // 迭代总次数+1
  iterationCounter++;

  Vec2<double> se_contactState(0, 0);

  for (int foot = 0; foot < 2; foot++)
  {
    // contactStates是二维向量，像素级操作
    double contactState = contactStates(foot);   //当前脚的支撑状态 所处的相位
    double swingState = swingStates(foot);       //当前脚的摆动状态 所处的相位

    std::cout << "swing " << foot << ": " << swingState << std::endl;
    std::cout << "Contact " << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;    //脚掌的世界坐标  没用上
    // 当该脚处于摆动状态时
    if (swingState > 0) // foot is in swing
    {
      // 从支撑状态转到摆动状态就视为是第一次摆动
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        // 设置从站立转为摆动的第一步 初始化摆动轨迹的起点 就是当前位置 给_p0赋值 
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      // 采用贝塞尔曲线计算腿的轨迹。根据起脚点、落地点和当前的相位(swingstate)来计算当前脚的理想位置和速度
      // swingState 当前脚的摆动状态所处的相位  以行走模式为例 swingTimes[0]=0.04*5=0.2
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      // 将三次方贝塞尔计算出来的值强转为double类型再给到pDesFootWorld 通过轨迹规划算得的脚在轨迹线上的位置
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // 判断左右腿，
      double side = -1.0 ;
      if (foot == 1)
      {
        // 右腿，原因是机器人坐标中右腿在y轴负半轴
        side = 1.0;
      }
      Vec3<double> hipOffset = {0, side*-0.02, -0.136};      //髋关节的偏移量
      // (脚轨迹位置-当前机器人原点位置):就是将脚位置转到“跟随世界坐标系” 再左乘rBody 就是转到机器人的坐标，
      // 减去髋关节偏移量是将起始点转到hip1所在的基坐标，y轴偏移量有待考证
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld); 

      if (vDesLeg.hasNaN()) //如果获得的速度无穷大 证明计算有误，无法到达，速度强制为0
      {
        vDesLeg << 0, 0, 0;
      }
      //将获得的目标位置和速度，以及对应的pd参数传给腿控制器。
      // 摆动腿由位置控制 ，前馈力由MPC计算，所以全强制为0
      data._legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      // 对脚位置的pd控制
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      // 单独对脚掌位置的pd控制，确保一直平行于地面
      data._legController->commands[foot].kptoe = 5; // 0
      data._legController->commands[foot].kdtoe = 0.1;
      se_contactState[foot] = contactState;
    }
    // 当该脚处于支撑状态时
    else if (contactState > 0) // foot is in stance
    { 
      // 从支撑状态转到摆动状态就视为第一次摆动
      firstSwing[foot] = true;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // (脚轨迹位置(位于踝关节正下方与地面接触的位置)-当前机器人原点位置):就是将脚位置转到“跟随世界坐标系” 再左乘rBody 就是转到机器人的坐标，
      // 减去髋关节偏移量是将起始点转到hip1所在的基坐标，此处偏移应该才是正确的偏移
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._biped->getHip2Location(foot);
      // (脚轨迹速度(位于踝关节正下方与地面接触的位置的速度)
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      if (vDesLeg.hasNaN())
        {
         vDesLeg << 0, 0, 0;
        }
      //将获得的目标位置和速度，以及对应的pd参数传给腿控制器。
      // 这是一个力位混控，但此处没有给pd值导致没有位置的控制输出，目前大趋势是力位混控，也就是前馈控制，
      // 在正常的位置控制闭环内，在收到干扰时将预测即(MPC)得到的力控制直接输入终端
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      // 对脚位置的pd控制 置0
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;
      // 单独对脚掌位置的pd控制，置0
      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;
      // 将MPC通过qp工具包 算得的前馈力赋值给腿控制器
      data._legController->commands[foot].feedforwardForce = f_ff[foot];
      // 保存此时所处的支撑状态相位 没用上
      se_contactState[foot] = contactState;

    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{
  //每10次迭代运行一次 iteration Counter迭代次数
  if ((iterationCounter % 10) == 0) 
  {
    // 加载 状态估计容器的处理结果数据
    auto seResult = data._stateEstimator->getResult();
    // 加载 期望状态命令
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data(); // 世界坐标位置
    double *v = seResult.vWorld.data(); // 世界坐标速度
    double *w = seResult.omegaWorld.data(); // 世界坐标角度
    double *quat = seResult.orientation.data(); // 四元数

    //Joint angles to compute foot rotation 关节角度q计算脚旋转
    // 临时变量 q 来加载10个电机的信息，将二维转为一维数组
    Eigen::Matrix<double, 10, 1> q;
      for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 5; k++)
      {
        q(i * 5 + k) = data._legController->data[i].q(k);
      }
    }
    // 临时指针，获取电机数据的起始地址
    double *joint_angles = q.data();

    double PI = 3.14159265359;
    //Joint angles offset correction 
    // 腿的角度，使其呈现稳定行走的状态，刚开始所有的关节坐标是全部平行于世界坐标的直膝状态，需要转为正常站立屈膝的基础上再旋转
    q(2) +=  0.3*PI;
    q(3) -=  0.6*PI;
    q(4) +=  0.3*PI;

    q(7) +=  0.3*PI;
    q(8) -=  0.6*PI;
    q(9) +=  0.3*PI;

    double PI2 = 2*PI;
    for(int i = 0; i < 10; i++){
      // fmod 函数用于计算两个浮点数的余数，确保在2PI以内
      q(i) = fmod(q(i) , PI2);
    }

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      // 抬脚点的三轴位置 减去 传感器获得的整体位置(可理解为质心位置)，并转为一维数组
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }
    //MPC Weights MPC各个输入量的权重
    double Q[12] = {70, 70, 0,  200, 100, 300,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    double *weights = Q;
    double *Alpha_K = Alpha;
    // yaw轴由状态估计器的解算结果获得  即imu获得
    double yaw = seResult.rpy[2];

    std::cout << "current position: " << p[0] << "  "<< p[1] << "  "<< p[2] << std::endl;

    // 06 x轴期待的运动速度 07 y轴期待的运动速度 由遥控器获取 z轴强制为零
    v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;
    // rBody只是一个3x3的姿态变换矩阵，因此只是将机器人自身在imu坐标下的遥控器速度转到平行于真实世界坐标，且原点与imu坐标重合跟随机器人移动的“跟随世界坐标”
    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    // 位置误差最大允许值
    const double max_pos_error = .15;

    double xStart = world_position_desired[0]; 
    double yStart = world_position_desired[1];
    // 修正xy轴的位置偏差 *p = seResult.position.data();
    // 世界坐标位置，由于world_position_desired是由速度对时间积分得到，而p是直接由位置传感器获得，所以会有偏差
    // 以位置传感器为基准，允许有最大误差
    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error; 
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;
    // 修正xy轴的位置偏差
    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    // Vec3<double> ori_des_world; 
    // 期待的世界角度 orientation desirable world 由遥控器命令输入转为三轴的角度 位置 角速度 速度  然后预测在现在的状态下未来十个MPC循环的三轴的角度 位置 角速度 速度
    ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];    
    // 轨迹数据的初始化
    double trajInitial[12] = {/*rpy_comp[0] + */stateCommand->data.stateDes[3],  // 0 遥控器命令输入的roll轴角度
                              /*rpy_comp[1] + */stateCommand->data.stateDes[4],  // 1 遥控器命令输入的pitch轴角度
                              seResult.rpy[2]*0,                                 // 2 强制为0的yaw轴角度
                              xStart*0,                                          // 3 强制为0的期待世界位置x轴
                              yStart*0,                                          // 4 强制为0的期待世界位置y轴
                              0.5 ,                                              // 5 强制为0.5的期待世界位置z轴
                              0,                                                 // 6 强制为0的roll轴的角度变化率
                              0,                                                 // 7 强制为0的pitch轴的角度变化率
                              stateCommand->data.stateDes[11],                   // 8 遥控器命令输入的yaw轴的角度变化率
                              v_des_world[0],                                    // 9 “跟随世界坐标”的x轴速度
                              v_des_world[1],                                    // 10“跟随世界坐标”的y轴速度
                              0};                                                // 11强制为0的z轴速度
    // 将12个轨迹数据 扩充10倍，给预测的10步用
    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];
      // 第0步需要强制为当前状态的欧拉角和位置
      if(i == 0) // start at current position  TODO consider not doing this
      {
        trajAll[0] = seResult.rpy[0];
        trajAll[1] = seResult.rpy[1];
        trajAll[2] = seResult.rpy[2];
        trajAll[3] = seResult.position[0];
        trajAll[4] = seResult.position[1];
        trajAll[5] = seResult.position[2];
      }
      else
      {
        // 如果没有x轴速度，即遥控器没有给出x轴速度命令
        if (v_des_world[0] == 0) {
        // 预测第i步的x轴位置 = 当前x轴位置 + i*0.04(一个MPC周期的时间) * “跟随世界坐标”的x轴速度， 即等于 当前x轴位置
        trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else{
        // 预测第i步的x轴位置 = 世界里程计获得的位置 + i*0.04(一个MPC周期的时间) * “跟随世界坐标”的x轴速度
         trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
        }
        if (v_des_world[1] == 0) {
        trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else{
         trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
        }
        //  yaw轴的角度变化率 遥控器输入
        if (stateCommand->data.stateDes[11] == 0){
        // 如果没有命令输入 就保持原来角度
        trajAll[12*i + 2] = trajInitial[2];
         }
        else{
        trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
      std::cout << "traj " << i << std::endl;
      for (int j = 0; j < 12; j++) {
        std::cout << trajAll[12 * i + j] << "  ";
      }
          std::cout<< " " <<std::endl;

    }

    //MPC Solver Setup
    dtMPC = dt * iterationsBetweenMPC; // 0.001*40
    // dtMPC=0.04 horizonLength=10
    setup_problem(dtMPC, horizonLength, 0.25, 500);
    
    //Solve MPC，使用时间函数计算进行一次MPC计算需要多长时间
    Timer t_mpc_solve;
    t_mpc_solve.start();
    update_problem_data(p, v, quat, w, r, joint_angles ,yaw, weights, trajAll, Alpha_K, mpcTable);
    printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());

    //Get solution and update foot forces    
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF; // Ground Reaction Force 地面反作用力
      Vec3<double> GRF_R; // 身体坐标下的脚所需要的力 
      Vec3<double> GRM; // Ground Reaction Moment 地面反作用力矩
      Vec3<double> GRM_R; // 身体坐标下的脚所需要的力矩
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      // 将地面反作用力 转到身体坐标下，但由于 地面反作用力方向 与 脚支撑力方向 相反所以要加负号
      GRF_R = - seResult.rBody * GRF; 
      // 将地面反作用力矩 转到身体坐标下，但由于 地面反作用力矩方向 与 脚支撑力矩方向 相反所以要加负号
      GRM_R = - seResult.rBody * GRM;
      std::cout << "RBody: " << seResult.rBody << std::endl;

      for (int i = 0; i < 3; i++){
        f(i) = GRF_R(i);
        f(i+3) = GRM_R(i);
      }
      // feedforward Force 前馈力 将算得力直接给到脚
      f_ff[leg] = f;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

  // void ConvexMPCLocomotion::GenerateTrajectory(int* mpcTable, ControlFSMData& data, bool omniMode, StateEstimate& _seResult){
    
  //   Vec3<double> v_des_world = _seResult.rBody.transpose() * v_des_robot;
  //   const double max_pos_error = .15;
  //   double xStart = world_position_desired[0];
  //   double yStart = world_position_desired[1];

  //   if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
  //   if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

  //   if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
  //   if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

  //   world_position_desired[0] = xStart;
  //   world_position_desired[1] = yStart;

  //   double trajInitial[12] = {ori_des_world[0],  // 0
  //                             ori_des_world[1],    // 1
  //                             0,    // 2
  //                             0,                                   // 3
  //                             0,                                   // 4
  //                             0.5 ,   // 5
  //                             0,                                        // 6
  //                             0,                                        // 7
  //                             stateCommand->data.stateDes[11],  // 8
  //                             v_des_world[0],                           // 9
  //                             v_des_world[1],                           // 10
  //                             0};                                       // 11

  //   for (int i = 0; i < horizonLength; i++)
  //   {
  //     for (int j = 0; j < 12; j++)
  //       trajAll[12 * i + j] = trajInitial[j];

  //     if(i == 0) // start at current position  TODO consider not doing this
  //     {
  //       trajAll[0] = seResult.rpy[0];
  //       trajAll[1] = seResult.rpy[1];
  //       trajAll[2] = seResult.rpy[2];
  //       trajAll[3] = seResult.position[0];
  //       trajAll[4] = seResult.position[1];
  //       trajAll[5] = seResult.position[2];
  //     }
  //     else
  //     {
  //       if (v_des_world[0] == 0) {
  //       trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
  //       }
  //       else{
  //        trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
  //       }
  //       if (v_des_world[1] == 0) {
  //       trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
  //       }
  //       else{
  //        trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
  //       }
  //       if (stateCommand->data.stateDes[11] == 0){
  //       trajAll[12*i + 4] = trajInitial[4];
  //        }
  //       else{
  //       trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
  //       //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
  //       }
  //     }
  //     std::cout << "traj " << i << std::endl;
  //     for (int j = 0; j < 12; j++) {
  //       std::cout << trajAll[12 * i + j] << "  ";
  //     }
  //         std::cout<< " " <<std::endl;

  //   }

  // }
