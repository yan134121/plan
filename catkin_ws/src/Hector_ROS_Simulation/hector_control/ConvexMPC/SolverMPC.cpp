#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "../third_party/qpOASES/include/qpOASES.hpp"
#include <stdio.h>
#include <sys/time.h>
#include "../include/common/Utilities/Timer.h"
#include <fstream>

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
// big enough to act like infinity, small enough to avoid numerical weirdness.// 大到可以表现得像无穷大，小到可以避免数字上的怪异。
#define NUM_VAR 12 // number of variables 变量的数量
#define NUM_CON 26 // number of constraints 约束条件的数量

RobotState rs;
// 标识符引入当前作用域 直接使用 无需指定命名空间
using Eigen::Dynamic; // Dynamic通常用于表示矩阵和数组中的动态大小维度。
using std::cout; 
using std::endl; // 换行并刷新
using std::ofstream; // 文件写入操作

// qpOASES::real_t a;

Matrix<fpt, Dynamic, 13> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 13, 12> Bdt;
Matrix<fpt, 13, 13> Adt;
Matrix<fpt, 25, 25> ABc, expmm;
Matrix<fpt, Dynamic, Dynamic> S; // 动态矩阵
Matrix<fpt, Dynamic, 1> X_d;
Matrix<fpt, Dynamic, 1> U_b;
Matrix<fpt, Dynamic, 1> L_b;
Matrix<fpt, Dynamic, Dynamic> fmat;
Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;
Matrix<fpt, NUM_CON, NUM_VAR> F_control;
Matrix<fpt, 1, 3> tx_F;
Matrix<fpt, 1, 3> ty_F;
Matrix<fpt, 1, 3> DNFG;

// Matrix<fpt,Dynamic,Dynamic> eye_12h;
Matrix<fpt, Dynamic, Dynamic> Alpha_diag;
Matrix<fpt, Dynamic, Dynamic> Alpha_rep;

// real_t:qpOASES 库中用于表示实数的类型
qpOASES::real_t *H_qpoases;
qpOASES::real_t *g_qpoases;

// A_qpoases(给到qp工具包) <-- fmat <-- F_control <-- GetConstraintMatrix <-- A_c矩阵 且没有改变
qpOASES::real_t *A_qpoases;
qpOASES::real_t *lb_qpoases;
qpOASES::real_t *ub_qpoases;
qpOASES::real_t *q_soln;

qpOASES::real_t *H_red;
qpOASES::real_t *g_red;
qpOASES::real_t *A_red;
qpOASES::real_t *lb_red;
qpOASES::real_t *ub_red;
qpOASES::real_t *q_red;
// 真正的分配
u8 real_allocated = 0;

char var_elim[2000];
char con_elim[2000]; //用于记录元素是否被删除或标记为某种状态

Matrix<fpt, 3, 3> euler_to_rotation(fpt roll, fpt pitch, fpt yaw) {
    
    fpt r = roll;
    fpt p = pitch;
    fpt y = yaw;

    // Calculate rotation matrix 计算旋转矩阵 绕xyz三轴旋转的旋转矩阵 对应rpy
    Matrix<fpt, 3, 3> Rx, Ry, Rz, Rb;
    Rx << 1, 0, 0,
          0, cos(r), -sin(r),
          0, sin(r), cos(r);
    Ry << cos(p), 0, sin(p),
          0, 1, 0,
          -sin(p), 0, cos(p);
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;
    Matrix<fpt, 3, 3> R;
    // 解释参考来源 http://t.csdnimg.cn/uxCbk
    R << 1, sin(r)*tan(p), cos(r)*tan(p),
          0, cos(r), -sin(r),
          0, sin(r)/cos(p), cos(r)/cos(p);  
    return R;
}


// Returns QP solution 获取QP问题解的q_soln
mfp* get_q_soln()
{
  return q_soln;
}
//用于判断一个浮点数 a 是否接近0
s8 near_zero(fpt a)
{
  return (a < 0.0001 && a > -.0001);
}
//用于判断一个浮点数 a 是否接近2
s8 near_one(fpt a)
{
  return near_zero(a - 2);
}
// 没用上
s8 is_even(int a)
{
  return is_even(fmod(a,2) == 0);
}

s8 is_odd(int a)
{
  return is_even(fmod(a,2) != 0);
}

/******************************************************************************************************/
/******************************************************************************************************/

// convert a 2D matrix to a 1D array 将2D矩阵转换为1D数组
void matrix_to_real(qpOASES::real_t *dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
//这段代码实现了将连续时域系统转换为离散时域二次规划问题的函数。
// Ac和Bc为上阶段获得的矩阵，对应公式(8)和(9)整理得来 dt=0.04 horizon=10
void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon) //二次规划
{
#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n"
       << Adt << "\nBdt:\n"
       << Bdt << endl;
#endif
  if (horizon > 19)
  {
    throw std::runtime_error("horizon is too long!");
  }
  //预测周期dt,对A矩阵离散化
  //     | 0_3x3 0_3x3 R_z=R_yaw 0_3x3      0 |
  // Ac= | 0_3x3 0_3x3 0_3x3     Identity() 0 |
  //     | 0_3x3 0_3x3 0_3x3     0_3x3      0 |
  //     | 0_3x3 0_3x3 0_3x3     0_3x3      0 |
  //     | 0     0     0         0        -1.f|
  Matrix<fpt, 13, 13> Acd = Matrix<fpt, 13, 13>::Identity() + dt * Ac;
  //对B矩阵离散化
  //     | 0_3x3             0_3x3               0_3x3 0_3x3 |
  // Bc= | 0_3x3             0_3x3               0_3x3 0_3x3 |
  //     |I_inv叉乘左脚r_feet I_inv叉乘右脚r_feet I_inv I_inv  |
  //     |Identity()/m       Identity()/m        0_3x3 0_3x3 |
  Matrix<fpt, 13, 12> Bcd = dt * Bc;

  for (int i = 0; i < 10; i++)
  {
    Eigen::Matrix<fpt, 13, 13> Acdm;
    Acdm = Matrix<fpt, 13, 13>::Identity();
    // 使用矩阵乘法，将Acd连乘，连乘0 1 2 3 4 ... 9，呈现下三角形状
    // Acdm=0; Acdm=Acd*Acd; Acdm=Acd*Acd*Acd; .... 
    for (int j = 0; j < i + 1; j++)
    {
      Acdm *= Acd;
    }
    // A_qp是一个动态行数固定列数(13)的矩阵，所以存放都是垂直往下存放
    // 开辟一个(13x13)的空间从空间头开始存放
    // [I A A^2 A^3 .... A^10]^T
    A_qp.block<13, 13>(i * 13, 0) << Acdm;
  }

  Eigen::Matrix<fpt, 13, 13> Acdp;
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < i + 1; j++)
    {
      Acdp = Matrix<fpt, 13, 13>::Identity();
      for (int k = 0; k < i - j; k++)
      {
        Acdp *= Acd;
      }

      if (i - j == 0)
      {
        Acdp = Matrix<fpt, 13, 13>::Identity();
      }

      B_qp.block<13, 12>(i * 13, j * 12) << Acdp * Bcd;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      B_qp.block<13, 12>(i * 13, j * 12) << Matrix<fpt, 13, 12>::Zero();
    }
  }

 #ifdef K_PRINT_EVERYTHING
  cout << "AQP:\n"
       << A_qp << "\nBQP:\n"
       << B_qp << endl;
 #endif
}

/******************************************************************************************************/
/******************************************************************************************************/

// Resizing & initaliztation: 调整大小和初始化:
void resize_qp_mats(s16 horizon)
{
  // 仅用于打印记录
  int mcount = 0;
  int h2 = horizon * horizon;
  // Eigen::NoChange 列数保持不变
  A_qp.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon * 1;

  B_qp.resize(13 * horizon, 12 * horizon);
  mcount += 13 * h2 * 12;

  S.resize(13 * horizon, 13 * horizon);
  mcount += 13 * 13 * h2;

  X_d.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon;

  U_b.resize(NUM_CON*horizon, Eigen::NoChange);
  mcount += NUM_CON*horizon;

  L_b.resize(NUM_CON*horizon, Eigen::NoChange);
  mcount += NUM_CON*horizon;

  fmat.resize(NUM_CON*horizon, 12*horizon);
  mcount += NUM_CON*12*h2;

  qH.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * h2;

  qg.resize(12 * horizon, Eigen::NoChange);
  mcount += 12 * horizon;

  // eye_12h.resize(12*horizon, 12*horizon);
  // mcount += 12*12*horizon;

  Alpha_rep.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  L_b.setZero();
  fmat.setZero();
  qH.setZero();
  // eye_12h.setIdentity();
  Alpha_rep.setZero();

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated)
  {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*NUM_CON*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*NUM_CON*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(NUM_CON*1*horizon*sizeof(qpOASES::real_t));
  mcount += NUM_CON*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(NUM_CON*1*horizon*sizeof(qpOASES::real_t));
  mcount += NUM_CON*horizon;
  q_soln = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*NUM_CON*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*NUM_CON*h2;
  lb_red = (qpOASES::real_t*)malloc(NUM_CON*1*horizon*sizeof(qpOASES::real_t));
  mcount += NUM_CON*horizon;
  ub_red = (qpOASES::real_t*)malloc(NUM_CON*1*horizon*sizeof(qpOASES::real_t));
  mcount += NUM_CON*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;

  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/

// Matrix Operations:
inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
  Matrix<fpt, 3, 3> cm;
  cm << 0.f, -r(2), r(1),
      r(2), 0.f, -r(0),
      -r(1), r(0), 0.f;
  return I_inv * cm;
}

/******************************************************************************************************/
/******************************************************************************************************/

// continuous time state space matrices.连续时间状态空间矩阵。
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 2> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13> &A, Matrix<fpt, 13, 12> &B)
{
  //存储状态空间方程中的状态矩阵 对应公式（8）论文中只有12x12的矩阵，这里多加了一行一列
  //    | 0_3x3 0_3x3 R_z=R_yaw 0_3x3      0 |
  // A= | 0_3x3 0_3x3 0_3x3     Identity() 0 |
  //    | 0_3x3 0_3x3 0_3x3     0_3x3      0 |
  //    | 0_3x3 0_3x3 0_3x3     0_3x3      0 |
  //    | 0     0     0         0        -1.f|
  A.setZero();
  A.block<3, 3>(0, 6) << R_yaw;  // 对应公式(5)(6)
  A.block<3, 3>(3, 9) << Matrix<fpt, 3, 3>::Identity();
  A.block<3, 1>(9, 12) << 0, 0, -1.f;
  // std::cout << "///////////// A:" << A << std::endl;
  //存储状态空间方程中的输入矩阵 对应公式（9）且对其进行扩充成13x12
  //    | 0_3x3             0_3x3               0_3x3 0_3x3 |
  //B=  | 0_3x3             0_3x3               0_3x3 0_3x3 |
  //    |I_inv叉乘左脚r_feet I_inv叉乘右脚r_feet I_inv I_inv  |
  //    |Identity()/m       Identity()/m        0_3x3 0_3x3 |
  B.setZero();
  // 得到转动惯量的逆
  Matrix<fpt, 3, 3> I_inv = I_world.inverse();
  // 没用上
  Matrix<fpt, 3, 3> Im;
  Im << 0, 0, 0,  0, 1.f, 0,  0, 0, 1.f;
  for (s16 b = 0; b < 2; b++)
  {
    // I_inv叉乘r_feet 矩阵的第 b 列
    B.block<3, 3>(6, b * 3) << cross_mat(I_inv, r_feet.col(b));
    // std::cout << "r_feet :" << r_feet.col(b) << std::endl;
  }
  B.block<3, 3>(6, 6) << I_inv ;
  B.block<3, 3>(6, 9) << I_inv ;  //switch
  B.block<3, 3>(9, 0) << Matrix<fpt, 3, 3>::Identity() / (m/1) ; //switch 
  B.block<3, 3>(9, 3) << Matrix<fpt, 3, 3>::Identity() / (m/1) ;
  // std::cout << "B:" << B << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1> &rpy)
{
  // from my MATLAB implementation

  // edge case!
  fpt as = t_min(2. * (q.w() * q.y() - q.x() * q.z()), .99999);
  rpy(0) = atan2(2.f * (q.w() * q.x() + q.y() * q.z()), 1. - 2.f * (sq(q.x()) + sq(q.y())));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1. - 2.f * (sq(q.y()) + sq(q.z())));
  // std::cout << "MPC solver rpy: " << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void print_problem_setup(problem_setup *setup)
{
  printf("DT: %.3f\n", setup->dt);
  printf("Mu: %.3f\n", setup->mu);
  printf("F_Max: %.3f\n", setup->f_max);
  printf("Horizon: %d\n", setup->horizon);
}

/******************************************************************************************************/
/******************************************************************************************************/

void print_update_data(update_data_t *update, s16 horizon)
{
  print_named_array("p", update->p, 1, 3);
  print_named_array("v", update->v, 1, 3);
  print_named_array("q", update->q, 1, 4);
  print_named_array("w", update->w, 1, 3);
  print_named_array("r", update->r, 3, 2);
  pnv("Yaw", update->yaw);
  print_named_array("weights", update->weights, 1, 12);
  print_named_array("trajectory", update->traj, horizon, 12);
  print_named_array("Alpha", update->Alpha_K, 1, 12);
  print_named_array("gait", update->gait, horizon, 2);
}

/******************************************************************************************************/
/******************************************************************************************************/

Matrix<fpt, 13, 1> x_0;
Matrix<fpt, 3, 3> I_world;
Matrix<fpt, 13, 13> A_ct;
Matrix<fpt, 13, 12> B_ct_r;

/******************************************************************************************************/
/******************************************************************************************************/

// Solve method
void solve_mpc(update_data_t *update, problem_setup *setup)
{
  //Joint angles to compute foot rotation
  Matrix<fpt, 10, 1> q;
  for(int i = 0; i < 10; i++)
  {
    q(i) = update->joint_angles[i];
  }

  double PI = 3.14159265359;
  //Joint angles offset correction
  // 逆时针旋转为正，因为建模时是直膝的，运行时是屈膝的
    q(2) +=  0.3*PI;
    q(3) -=  0.6*PI;
    q(4) +=  0.3*PI;

    q(7) +=  0.3*PI;
    q(8) -=  0.6*PI;
    q(9) +=  0.3*PI;
  //将所有关节角度限制在 [0, 2 * PI] 的范围内
    double PI2 = 2 * PI;     
    for (int i = 0; i < 10; i++) {
        q(i) = fmod(q(i), PI2);
    }
        
  // cout << "solver q:\n"
      //  << q << endl;
  // 将更新的状态数据设置进变量rs
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  rs.print();
  print_update_data(update, setup->horizon);
#endif

  // roll pitch yaw
  Matrix<fpt, 3, 1> rpy;
  // 自己创建的四元数转欧拉角 不使用库函数避免错误
  quat_to_rpy(rs.q, rpy);
  // Eigen::Matrix3d rot_mat;
  Matrix<fpt, 3, 3> Rb;
  std::cout << "rpy: " << rpy(0) << ", " << rpy(1) << ", " << rpy(2) << std::endl;
 // 欧拉角转为旋转矩阵
  Rb = euler_to_rotation(rpy(0), rpy(1), rpy(2));
  // 当前状态下的三轴的角度 位置 角速度 速度 用于MPC计算
  x_0 << rpy(0), rpy(1), rpy(2), rs.p, rs.w, rs.v, 9.81f;
  // 1、两个坐标系原点重合，姿态方向不同 如下 2、两个坐标系原点不重合，但姿态方向相同，I_1 = I_c + m(p_c.transpose() . p_c . 单位3x3矩阵 - p_c . p_c.transpose())
  I_world = rs.R * rs.I_body * rs.R.transpose(); // original
  // 将所需的参数填进A、B两个矩阵
  ct_ss_mats(I_world, 12.0, rs.r_feet, Rb, A_ct, B_ct_r);

#ifdef K_PRINT_EVERYTHING
  cout << "Initial state: \n"
       << x_0 << endl;
  cout << "World Inertia: \n"
       << I_world << endl;
  cout << "A CT: \n"
       << A_ct << endl;
  cout << "B CT (simplified): \n"
       << B_ct_r << endl;
#endif
  // QP matrices
  // dt=0.04 horizon=10
  // 将连续时间状态空间矩阵 A_ct 和输入矩阵 B_ct_r 转换为离散时间下的二次规划（QP）问题的矩阵
  // 
  c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);

  // weights
  Matrix<fpt, 13, 1> full_weight;
  // 将权重赋值进来
  // MPC Weights {70, 70, 0,  200, 100, 300,  1, 1, 1,  1, 1, 1} roll pitch yaw x y z droll dpitch dyaw dx dy dz
  for (u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i]; 
  full_weight(12) = 0.f;
  // full_weight是一个13x1的列向量，(replicate)将复制full_weight的horizon行 x 1列的数值给到S的对角(diagonal)元素
  //     |70                                |
  //     |   70                             | 
  //     |      0                           | 
  // S = |        200                       | 
  //     |            100                   |  
  //     |                300               |
  //     |                    1             | 
  //     |                      1           |
  //     |                        1         |
  //     |                          1       |
  //     |                            0     |
  //     |                              0.f |
  S.diagonal() = full_weight.replicate(setup->horizon, 1);

  // trajectory 轨迹
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 12; j++)
      // 将预测到的未来120个轨迹 10次MPC周期(三轴角度 位置 角速度 速度)赋值进来，而 traj 有432个存储位置
      // X_d是一个动态行 1列的列向量 由于前面使用的是13的倍数，所以每个周期都会留下最后一个0 这个位置就是重力加速度g
      X_d(13 * i + j, 0) = update->traj[12 * i + j];
  }
 //  设置电机扭矩上限为33.5
  double motorTorqueLimit = 33.5;
 // horizonn = 10；
  int horizonn = setup->horizon;
  //resize创建一个大小为 2*horizonn x 1 的矩阵 gaitt，用于存储步态信息
  Eigen::MatrixXf gaitt;
  gaitt.resize(2*horizonn, 1);
  for(int i = 0; i < 2*horizonn; i++){
    // 将步态赋值进来 前20个
    gaitt(i,0) = update->gait[i]; 
  }

  //Constraint Calculation 约束计算 
  // 机器人状态 rs、关节角度 q、控制水平时间步数 setup->horizon、约束的数量 NUM_CON、电机扭矩限制 motorTorqueLimit、大数值 BIG_NUMBER、最大力 setup->f_max 、步态信息 gaitt
  Constraints constraints(rs, q, setup->horizon, NUM_CON, motorTorqueLimit, BIG_NUMBER, setup->f_max, gaitt);
  U_b = constraints.GetUpperBound();
  L_b = constraints.GetLowerBound();

  // 获取 A_c 约束矩阵 (26x12的矩阵) 前13行是对左腿的约束，后13行是对右腿的约束
  F_control = constraints.GetConstraintMatrix();
  // Set to fmat QP
  for(s16 i = 0; i < setup->horizon; i++)
  {
    // 表示从 fmat 的第 i*26 行、第 i*12 列开始，开辟一个大小为 26 x 12 的矩阵块。存放脚的控制系数,约束矩阵系数
    // 实际上开辟了10*26行 10*12列的空间，由于F_control新数据赋值在旧数据的右下角，呈现阶梯状，所以有大量的零，
    // A_qpoases(给到qp工具包) <-- fmat <-- F_control <-- GetConstraintMatrix <-- A_c矩阵 且没有改变
    fmat.block(i*NUM_CON,i*12,NUM_CON,12) = F_control;
  }
  // Construct K: 创建一个大小为 12 x 12 的对角矩阵
  Alpha_diag.resize(12, 12);
  Alpha_diag.setZero();

  for (s16 i = 0; i < 12; i++)
  {
    // 填充第i行i列的1行1列个元素，也就是对角线元素填充 来源 Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    Alpha_diag.block(i, i, 1, 1) << update->Alpha_K[i];
  }
  for (s16 i = 0; i < setup->horizon; i++)
  {
    // 一次填充12个对角线元素，填充10次
    Alpha_rep.block(i * 12, i * 12, 12, 12) << Alpha_diag;
  }
  // Equivalent to Matlab Formulaion 见文章公式(24~25)
  qH = 2 * (B_qp.transpose() * S * B_qp + Alpha_rep);
  // (A_qp * x_0)是预测10个周期中x(k+1)=Ax(k)+Bu(k)中等式右边的第一项,A_qp是比A多了一行重力加速度g,
  // X_d是按照现在遥控器给的命令预测出的未来10个MPC共13*10行1列的状态列向量(三轴角度 位置 角速度 速度 重力加速度g) 注意是在没有u(x)输入的情况下,所以不会涉及B矩阵
  // 相减就得到[I A A^2 A^3 .... A^10]^T
  qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

  // Calls function that sets parameters matrices in qpOASES types
  // 将二维的矩阵转为一维的行向量(数组)
  matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
  matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
  // 转为一维行向量 10*12个元素一周期 有10*26个周期
  matrix_to_real(A_qpoases,fmat,setup->horizon*NUM_CON, setup->horizon*12);
  matrix_to_real(ub_qpoases,U_b,setup->horizon*NUM_CON, 1);
  matrix_to_real(lb_qpoases,L_b,setup->horizon*NUM_CON, 1);

// 约束的数量 26*10 在 A_c矩阵约束中，行代表约束条件的数量，列代表要约束变量的数量
  s16 num_constraints = NUM_CON*setup->horizon;
  // 变量的数量 12*10
  s16 num_variables = 12*setup->horizon;

  // Max # of working set recalculations 工作集重新计算的最大次数
  //最大工作集重算次数，用于控制QP（Quadratic Programming）求解器的迭代次数
  qpOASES::int_t nWSR = 500;
  // 新的变量的数量 12*10
  int new_vars = num_variables;
  //  新的约束的数量 26*10 
  int new_cons = num_constraints;
//  创建一个数组用于存储要消除的约束 初始化为零 constraints eliminate
  for (int i = 0; i < num_constraints; i++)
    con_elim[i] = 0;

// 创建一个数组用于存储要消除的变量 初始化为零 variables eliminate
  for (int i = 0; i < num_variables; i++)
    var_elim[i] = 0;
  // 循环260周期
  for (int i = 0; i < num_constraints; i++)
  {
    // 上下限都不接近于0就跳过该次for循环 所以只有在左右腿 z轴摩擦力的不等式约束 这一条，才会出现上下限都为零的情况，此时为零代表该腿没有支撑力不用计算，要计算另一条腿
    // 即U_b(12+26i) U_b(25+26i) L_b(12+26i) L_b(25+26i) 而且单个周期即26内只会出现左腿或右腿，不会同时出现两，假设左腿限迈出，
    // 所以260次只在12(左腿) 12+26+13(右腿) 12+26+13+13(左腿) 12+26+13+13+26+13(右腿) 12+26+13+13+26+13+13(左腿) ...五左 五右 该for循环真正运行只有10次
    if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;

    // A_qpoases 10*12个元素为原来大矩阵的一行 原来有10*26行 这里一次取120个元素  获取大约束矩阵(260*120)中的当前行 取出120个
    // 由于 z轴摩擦力的不等式约束 这一条约束位于每条腿13个约束的最后一个，当上面判断为零时的i正好到了这个约束位置，
    // 下一条约束就是需要计算支撑力腿的开始，所以 i * num_variables 就是需要计算支撑力腿的开始 num_variables(即120)是因为大矩阵260*120是一个阶梯状矩阵，
    // 转成一维的A_qpoases后每120元素才是原来大矩阵的一行
    double *c_row = &A_qpoases[i * num_variables]; 
    // 循环拿到的120个元素 但会有大量的零，
    for (int j = 0; j < num_variables; j++)
    {
      // 检查每个元素 是否有等于2的，从而判断是选择哪一条腿，每13次i才会判断到一次
      // 因为在第2列和第5列才会有2.f，且经过horizon行列同时扩充10倍，所以会有20次机会进来分别是是j=2,5+12, 2+12+12，....
      if (near_one(c_row[j]))
      {
          new_vars -= 6; // 开始是120 每次减6，刚好进来10次  最后剩一半60
          new_cons -= 13; // 开始是260 每次减13， 刚好进来10次  最后剩一半130

          int cs;
          //偶数 对应leg1左腿，2,2+12,2+24,2+36....
          if (j%2 == 0){
            cs = (j+4)/6*13-1; // (2+4)/6*13-1 =12   (2+12+4)/6*13-1 =38 差值正好是26行
          }
          else{ // 奇数 对应leg2右腿 5,5+12,5+24,5+36,...
            cs = (j+1)/6*13+12; // (5+1)/6*13+12 =25 (5+12+1)/6*13+12 = 38 差值正好是26行
          }
          // 要消除的变量,列的部分 从而选择左右腿 被标记的是不要计算的腿，即摆动腿
          var_elim[j+6] = 1; // 在M_lz或M_rz位置标记1
          var_elim[j+5] = 1; // 在M_ly或M_ry位置标记1
          var_elim[j+4] = 1; // 在M_lx或M_rx位置标记1
          var_elim[j-2] = 1; // 在F_lx或F_rx位置标记1
          var_elim[j-1] = 1; // 在F_ly或F_ry位置标记1
          var_elim[j  ] = 1; // 在F_lz或F_rz位置标记1
          // 要消除的约束，行的部分 如果j是偶数那么leg1全部标记为1，即前13行 选择前13行的左腿或后13行的右腿
          con_elim[cs-0] = 1; 
          con_elim[cs-1] = 1;
          con_elim[cs-2] = 1;
          con_elim[cs-3] = 1;
          con_elim[cs-4] = 1;         
          con_elim[cs-5] = 1;
          con_elim[cs-6] = 1;
          con_elim[cs-7] = 1;
          con_elim[cs-8] = 1;
          con_elim[cs-9] = 1;         
          con_elim[cs-10] = 1;
          con_elim[cs-11] = 1;
          con_elim[cs-12] = 1;
      }
    }
  }
  
  //从原始的QP问题中提取出新的子问题，以减小问题规模
  if (1 == 1)
  {
    // 标记变量 variables indicate new_vars=60
    int var_ind[new_vars];
    // 标记约束 constraints indicate new_cons=130
    int con_ind[new_cons];
    int vc = 0;
    // 循环120次，也就是读完大矩阵(260行120列)里的一行
    for (int i = 0; i < num_variables; i++)
    {
      // 提取需要保留的变量索引号 
      // 如果 消除变量variables eliminate 数组里的值不等于1 即没有被标记
      // 如果 var_elim[i] = 0;   var_ind[]=i 将记录它的序号
      if (!var_elim[i])
      {
        if (!(vc < new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    // 提取需要保留的约束索引号
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      // constraints eliminate 消除约束
      // 如果 con_elim[i] = 0; con_ind[]=i 将记录它的序号
      if (!con_elim[i])
      {
        if (!(vc < new_cons))
        {
          printf("BAD ERROR 2\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }
    for (int i = 0; i < new_vars; i++)
    {
      // 提取子问题的g矩阵
      // 有用的数值序号赋值给 olda
      int olda = var_ind[i];
      // 把有用的数据重现排成  g_red[]
      g_red[i] = g_qpoases[olda];
      //提取子问题的H矩阵 new_vars = 60
      for (int j = 0; j < new_vars; j++)
      {
        //有用的数值序号赋值给 olda
        int oldb = var_ind[j];
        H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
      }
    }

    //提取子问题的约束矩阵 new_cons=130
    for (int con = 0; con < new_cons; con++)
    {
      // new_vars = 60
      for (int st = 0; st < new_vars; st++)
      {
        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
        A_red[con * new_vars + st] = cval;
      }
    }
    //提取子问题的上下界
    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }
    //计时器初始化 ,用于记录求解QP问题所需的时间
    Timer solve_timer;

    // qpOASES problem new_vars = 60 new_cons=130
    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    //设置为MPC（Model Predictive Control）模式，关闭了打印输出
    op.setToMPC();
    // op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    // int_t nWSR = 50000;

    // QP initialized
    //rval 变量用于存储初始化的返回值
    // 这是使用qp工具箱所需要初始化的参数 
    // 二次项矩阵 H_red,  一次项列向量 g_red,  约束的系数矩阵 A_red,  等式约束的下界 NULL,
    // 等式约束的上界 NULL,不等式约束的下界 lb_red, 不等式约束的上界 ub_red,   最大重算次数nWSR
    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    (void)rval;
    //rval2 变量存储了求原解的返回值，即SUCCESSFUL_RETURN 成功 或RET_QP_NOT_SOLVED 没有解
    // q_red 保存求解结果
    int rval2 = problem_red.getPrimalSolution(q_red);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");
    // 打印一次计算qp所需要的时间，和所使用的变量个数 和约束个数
    printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

    // Reformats solution and stores into q_red 重新格式化解决方案并存储到q_red中
    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      // 判断是否为被标记的变量
      if (var_elim[i])
      {
        q_soln[i] = 0.0f;
      }
      else
      {
        // 将求解结果赋值到对应未被标记的位置
        q_soln[i] = q_red[vc];
        vc++;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout<<"fmat:\n"<<fmat<<endl;
#endif
}