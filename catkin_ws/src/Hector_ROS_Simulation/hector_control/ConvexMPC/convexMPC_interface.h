#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36  // K最大步态分段数

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt; // 时间步长（delta time），即模拟中每个时间步的持续时间
  float mu; // 摩擦系数的倒数
  float f_max; // 最大力或最大力矩 // 500
  int horizon; // 规划/控制的时域长度 设置的是10
};

struct update_data_t
{
  float p[3]; // 三轴位置
  float v[3]; // 三轴速度
  float q[4]; // 四元数
  float w[3]; // 角速度
  float r[6]; // 两只脚的坐标位置
  float joint_angles[10]; // 关节角度
  float yaw;
  float weights[12]; // 权重
  float traj[12*K_MAX_GAIT_SEGMENTS]; // 轨迹
  
  // float alpha;
  float Alpha_K[12];
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);

EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double* joint_angles, double yaw, double* weights, double* state_trajectory, double* Alpha_K, int* gait);

#endif