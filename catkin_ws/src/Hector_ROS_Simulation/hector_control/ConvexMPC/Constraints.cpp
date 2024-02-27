#include "Constraints.h"

// ====================== MPC Solver Constraints Class Implementation ======================= //
// MPC求解器约束类实现
/**
 * @brief 约束类的构造函数
 * @details Calculates upper bound for the QP 计算QP的上界
 *          参考资料http://t.csdnimg.cn/XXT6C
 * @param rs 机器人状态
 *        jointAngles 关节角度矩阵 
 *        horizon 规划/控制的时域长度 输入的是10
 *        num_constraints 约束的数量 输入的是26
 *        motorTorqueLimit 电机扭矩限制 输入的电机扭矩上限为33.5
 *        big_number 极大值，输入是5e10
 *        f_max 输入是500
 *        gait
 *        函数中使用 : 可以直接初始化不需要运算的成员，提高效率
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
Constraints::Constraints(RobotState& rs,                        const Eigen::MatrixXf& jointAngles,  int horizon,         int num_constraints, 
                         float motorTorqueLimit,                float big_number,                    float f_max,         const Eigen::MatrixXf& gait)
                       : rs_(rs),                               q(jointAngles),                      horizon_(horizon),   Num_Constraints(num_constraints), 
                         motorTorqueLimit_(motorTorqueLimit),   BIG_NUMBER_(big_number),             f_max_(f_max),       gait_(gait) {

    Num_Variables = 12; // {Fx1, Fy1, Fz1, Fx2, Fy2, Fz2, Mx1, My1, Mz1, Mx2, My2, Mz2}
    U_b = Eigen::VectorXf::Zero(horizon * Num_Constraints); // 260x1的列向量
    L_b = Eigen::VectorXf::Zero(horizon * Num_Constraints); // 260x1的列向量
    A_c = Eigen::MatrixXf::Zero(Num_Constraints, Num_Variables); // A_c是 26x12二维矩阵所以接收需要置零的行和列数量

    CalculateUpperBound();
    CalculateLowerBound();
    CalculateFootRotationMatrix();
    CalculateConstarintMatrix();
    
}

/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算上界约束
 * @details Calculates upper bound for the QP 计算QP的上界
 *          参考资料http://t.csdnimg.cn/XXT6C MIT四足机器狗MPC算法学习笔记
 *          U_b 是260x1的列向量
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
void Constraints::CalculateUpperBound() {
  for(s16 i = 0; i < horizon_; i++){
    // leg1
    U_b(0 + Num_Constraints*i) = BIG_NUMBER_;     //关于x轴正方向摩擦力约束的上限，因为摩擦力可以无穷大但不能为零不然打滑，所以给一个极大值避免计算超出
    U_b(1 + Num_Constraints*i) = BIG_NUMBER_;     //关于x轴负方向摩擦力约束的上限
    U_b(2 + Num_Constraints*i) = BIG_NUMBER_;     //关于y轴正方向摩擦力约束的上限
    U_b(3 + Num_Constraints*i) = BIG_NUMBER_;     //关于y轴负方向摩擦力约束的上限

    U_b(4 + Num_Constraints*i) = 0.01;            // 绕X轴的扭矩上限为0.01

    U_b(5 + Num_Constraints*i) = motorTorqueLimit_;
    U_b(6 + Num_Constraints*i) = motorTorqueLimit_;
    U_b(7 + Num_Constraints*i) = motorTorqueLimit_;   //电机转矩的最大值
    U_b(8 + Num_Constraints*i) = motorTorqueLimit_;      
    U_b(9 + Num_Constraints*i) = motorTorqueLimit_;

    U_b(10 + Num_Constraints*i) = BIG_NUMBER_;      
    U_b(11 + Num_Constraints*i) = BIG_NUMBER_;

    U_b(12 + Num_Constraints*i) = f_max_ * gait_(2*i + 0) ;         //关于z轴方向上的力约束上限 = 500*步态中的奇数位
    
    // leg2 
    U_b(13 + Num_Constraints*i) = BIG_NUMBER_;
    U_b(14+ Num_Constraints*i) = BIG_NUMBER_;
    U_b(15 + Num_Constraints*i) = BIG_NUMBER_;
    U_b(16 + Num_Constraints*i) = BIG_NUMBER_;
    
    U_b(17 + Num_Constraints*i) = 0.01;

    U_b(18 + Num_Constraints*i) = motorTorqueLimit_;
    U_b(19 + Num_Constraints*i) = motorTorqueLimit_;
    U_b(20 + Num_Constraints*i) = motorTorqueLimit_;
    U_b(21 + Num_Constraints*i) = motorTorqueLimit_;      
    U_b(22 + Num_Constraints*i) = motorTorqueLimit_;

    U_b(23 + Num_Constraints*i) = BIG_NUMBER_;      
    U_b(24 + Num_Constraints*i) = BIG_NUMBER_;

    U_b(25 + Num_Constraints*i) = f_max_ * gait_(2*i + 1); // f_max_ = 500
  }

}

/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算下界约束
 * @details Calculates lower bound for the QP 计算QP的下界
 *          L_b 是260x1的列向量
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
void Constraints::CalculateLowerBound() {
  // QP Lower Bound
  for(s16 i = 0; i < horizon_; i++){
    // leg1
      L_b(0 + Num_Constraints*i) = 0.0f;   //关于x轴正方向摩擦力约束的下限
      L_b(1 + Num_Constraints*i) = 0.0f;   //关于x轴负方向摩擦力约束的下限
      L_b(2 + Num_Constraints*i) = 0.0f;   //关于y轴正方向摩擦力约束的下限
      L_b(3 + Num_Constraints*i) = 0.0f;   //关于y轴负方向摩擦力约束的下限

      L_b(4 + Num_Constraints*i) = 0.0f;
      L_b(5 + Num_Constraints*i) = 0.0f;      
      L_b(6 + Num_Constraints*i) = 0.0f;

      L_b(7 + Num_Constraints*i) = -motorTorqueLimit_;       //电机转矩最小值限制
      L_b(8 + Num_Constraints*i) = -motorTorqueLimit_;
      L_b(9 + Num_Constraints*i) = -motorTorqueLimit_;
      L_b(10 + Num_Constraints*i) = -motorTorqueLimit_;      
      L_b(11 + Num_Constraints*i) = -motorTorqueLimit_;

      L_b(12 + Num_Constraints*i) = 0.0f ;

      // leg2 
      L_b(13 + Num_Constraints*i) = 0.0f;
      L_b(14 + Num_Constraints*i) = 0.0f;
      L_b(15 + Num_Constraints*i) = 0.0f;
      L_b(16 + Num_Constraints*i) = 0.0f;
      
      L_b(17 + Num_Constraints*i) = 0.0f;
      L_b(18 + Num_Constraints*i) = 0.0f;      
      L_b(19 + Num_Constraints*i) = 0.0f;

      L_b(20 + Num_Constraints*i) = -motorTorqueLimit_;
      L_b(21 + Num_Constraints*i) = -motorTorqueLimit_;
      L_b(22 + Num_Constraints*i) = -motorTorqueLimit_;
      L_b(23 + Num_Constraints*i) = -motorTorqueLimit_;      
      L_b(24 + Num_Constraints*i) = -motorTorqueLimit_;

      L_b(25 + Num_Constraints*i) = 0.0f;
  }    
}

/**
 * @brief 计算约束矩阵
 * @details Calculates constraint matrix for the 
 *          QP计算QP的约束矩阵
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
void Constraints::CalculateConstarintMatrix() {
    fpt mu = 5.0; // 摩擦系数的倒数 µ = 0.2
    fpt lt = 0.09; // l_toe 脚前掌长度
    fpt lh = 0.06; // l_heel 脚后跟长度
    // 给该1x3的行向量起名为Moment_selection(力矩选择)，且只选择X轴
    Matrix<fpt,1,3> Moment_selection(1.f, 0, 0); 
    Matrix<fpt,1,3> lt_vec; 
    Matrix<fpt,1,3> lt_3D;
    lt_vec << 0, 0, lt;  // 只对z轴有效

    Matrix<fpt,1,3> lh_vec;
    Matrix<fpt,1,3> lh_3D;
    lh_vec << 0, 0, lh;

    Matrix<fpt,1,3> M_vec;
    M_vec << 0, 1.0, 0;   // 只对y轴有效
    Matrix<fpt,1,3> M_3D;

/*****************************************************************************************************************************************************************
    A_qpoases(给到qp工具包) <-- fmat <-- F_control <-- GetConstraintMatrix <-- A_c矩阵 且中途没有发生没有改变
 * 下面是A_c(26x12的矩阵)形式，矩阵的前13行是对左腿的约束，后13行是对右腿的约束，每一行就是一个约束，每一行的约束里面一共有12个对象需要约束，采用这种有大量0的冗余形式是为了写成矩阵
 * 用交换律和摩擦系数倒数，使得只需要约束下界，即0, A_c只是约束系数矩阵，上下界在U_b和L_b
 * 在 A_c矩阵约束中，行代表约束条件的数量，列代表要约束变量的数量!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  世界坐标系下
 * 
 *                     左腿xyz三轴力，  右腿xyz三轴力，  左腿xyz三轴力矩，右腿xyz三轴力矩
 *                    F_lx F_ly F_lz  F_rx F_ry F_rz  M_lx M_ly M_lz  M_rx M_ry M_rz                 左腿的约束条件 对应的论文公式(17)~(19)
 *                    -mu   0   1.f    0    0    0     0    0    0     0    0    0     (0行)          F_ix ≤ µF_iz 和 mu = -(1/µ) --> -mu*F_ix + F_iz ≥ 0  
 *                          .               .               .               .          (1行)        −µF_iz ≤  F_ix                -->  mu*F_ix + F_iz ≥ 0
 *                          .               .               .               .          (2行)          F_iy ≤ µF_iz                --> -mu*F_iy + F_iz ≥ 0
 *                          .               .               .               .          (3行)        −µF_iz ≤  F_iy                -->  mu*F_iy + F_iz ≥ 0
 *                          .               .               .               .          (4行)         计算左腿绕X轴的扭矩，上下界限制在0-0.01                                                              
 *                          .               .               .               .          (5行)         5-9行 单条腿五个电机扭矩约束                                              
 *                          .               .               .               .          (6行)                                                                      
 *                          .               .               .               .          (7行)                                                                      
 *                          .               .               .               .          (8行)                                                                      
 *                          .               .               .               .          (9行)                                                                              
 *                          .               .               .               .          (10行)        限制Fz * 前半部分脚掌距离得到的绕y轴扭矩 + 左脚绕Y轴扭矩之和  注意方向                                                            
 *                          .               .               .               .          (11行)        限制Fz * 后半部分脚掌距离得到的绕y轴扭矩 + 左脚绕Y轴扭矩之和  注意方向                                                              
 *                     0    0   2.f     0   0   0      0    0    0     0    0    0     (12行)        限制左脚Z轴方向上的力的大小，系数设置为2是为了作为后面消除计算的标记数字    
 *A_c(26x12的矩阵) =                                          
 *                    F_lx F_ly F_lz  F_rx F_ry F_rz  M_lx M_ly M_lz  M_rx M_ry M_rz                 右腿的约束条件 对应的论文公式
 *                                                                                     (13行)          F_ix ≤ µF_iz 和 mu = -(1/u) --> -mu*F_ix + F_iz ≥ 0
 *                          .               .               .               .          (14行)        −µF_iz ≤  F_ix                -->  mu*F_ix + F_iz ≥ 0
 *                          .               .               .               .          (15行)          F_iy ≤ µF_iz                --> -mu*F_iy + F_iz ≥ 0
 *                          .               .               .               .          (16行)        −µF_iz ≤  F_iy                -->  mu*F_iy + F_iz ≥ 0
 *                          .               .               .               .          (17行)        计算右腿绕X轴的扭矩，上下界限制在0-0.01                                                                    
 *                          .               .               .               .          (18行)        18-22行 单条腿五个电机扭矩约束                                   
 *                          .               .               .               .          (19行)                                                                         
 *                          .               .               .               .          (20行)                                                                         
 *                          .               .               .               .          (21行)                                                                         
 *                          .               .               .               .          (22行)                                                                              
 *                          .               .               .               .          (23行)         限制Fz * 前半部分脚掌距离得到的绕y轴扭矩 + 左脚绕Y轴扭矩之和  注意方向                                                               
 *                          .               .               .               .          (24行)         限制Fz * 后半部分脚掌距离得到的绕y轴扭矩 + 左脚绕Y轴扭矩之和  注意方向                                                               
 *                     0    0    0      0   0   2.f     0   0     0     0   0    0     (25行)         限制右脚Z轴方向上的力的大小                                                                     
************************************************************************************************************************************************************************/
    // leg 1
    // 由于A_c是动态的二维矩阵，该语法为：提取一个子块，这个子块的大小是1×12，起始位置是矩阵的左上角（第 0 行，第 0 列），并赋值
    // 约束x轴正方向摩擦力
    A_c.block<1, 12>(0, 0) //Friction leg 1
        << -mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;     
    // 约束x轴负方向摩擦力 
    A_c.block<1, 12>(1, 0)
        << mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;         //mu*fx+fz<U_b
    // 约束y轴正方向摩擦力
    A_c.block<1, 12>(2, 0)
        << 0, -mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;    //-mu*fy+fz<U_b
    // 约束y轴负方向摩擦力 
    A_c.block<1, 12>(3, 0)
        << 0,  mu, 1.f,  0, 0, 0, 0, 0, 0, 0, 0, 0;        //-mu*fy+fz<U_b
    // 约束绕X轴的扭矩
    A_c.block<1, 12>(4, 0) //Mx Leg 1              
        << 0, 0, 0,  0, 0, 0,  Moment_selection * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
    // F_control.block<5, 3>(5, 0) 
    //     << Jv_L.transpose() * rs.R.transpose();           
    // F_control.block<5, 3>(5, 6) 
    //     << Jw_L.transpose() * rs.R.transpose();    
    A_c.block<1, 12>(10, 0) //Line Leg 1      
        // 只有姿态变换，没有位移，两个旋转矩阵的转置将在世界坐标系下脚的力转到脚坐标系，该坐标系的原点应该是和身体坐标系的原点重合，
        // 因为在支撑状态下脚坐标系和身体坐标系的x轴没有偏移，对计算转矩没有影响，加之该模型是单刚体模型 所以可以简化运算，只使用旋转矩阵，等效在同一个位置
        // 由于两只脚的地面反作用力 点 是等价在踝关节的正下方与地面的交点，Fz * 前脚掌长度 = 绕y轴的扭矩，所以可以相加
        // 力矩的方向是 右手定则：r沿着四指方向，四指转向F方向，这时候大拇指就是力矩方向，所以在前后脚掌加负号
        << -lt_vec * R_foot_L.transpose()* rs_.R.transpose(),   0, 0, 0,  M_vec * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
    A_c.block<1, 12>(11, 0)        
        <<  -lh_vec * R_foot_L.transpose()* rs_.R.transpose(),   0, 0, 0, -M_vec * R_foot_L.transpose()* rs_.R.transpose(),  0, 0, 0;
    A_c.block<1, 12>(12, 0) //Fz Leg 1
        << 0, 0, 2.f, 0, 0, 0,   0, 0, 0, 0, 0, 0;       //2*fz<U_b 两倍z轴的力要在限制范围内
  

    //leg 2
    A_c.block<1, 12>(13, 0) //Friction leg 2
        <<  0, 0, 0,  -mu, 0, 1.f,    0, 0, 0,   0, 0, 0;
    A_c.block<1, 12>(14, 0)
        <<  0, 0, 0,   mu, 0, 1.f,    0, 0, 0,   0, 0, 0;
    A_c.block<1, 12>(15, 0)
        <<  0, 0, 0,    0, -mu, 1.f,  0, 0, 0,   0, 0, 0;
    A_c.block<1, 12>(16, 0)
        <<  0, 0, 0,    0, mu, 1.f,   0, 0, 0,   0, 0, 0;        
    A_c.block<1, 12>(17, 0) //Mx Leg 1
        << 0, 0, 0,  0, 0, 0,  0, 0, 0,  Moment_selection * R_foot_R.transpose()* rs_.R.transpose();
    // F_control.block<5, 3>(5, 3) 
    //     << Jv_R.transpose() * rs.R.transpose();
    // F_control.block<5, 3>(5, 9) 
    //     << Jw_R.transpose() * rs.R.transpose();  
    A_c.block<1, 12>(23, 0) //Line Leg 2
        << 0, 0, 0,     -lt_vec * R_foot_R.transpose()* rs_.R.transpose(), 0, 0, 0,    M_vec * R_foot_R.transpose()* rs_.R.transpose();
    A_c.block<1, 12>(24, 0)
        << 0, 0, 0,    -lh_vec * R_foot_R.transpose()* rs_.R.transpose(),  0, 0, 0,   M_vec * R_foot_R.transpose()* rs_.R.transpose();  
    A_c.block<1, 12>(25, 0)  //Fz Leg 2
        << 0, 0, 0, 0, 0, 2.f, 0, 0, 0, 0, 0, 0;
}

/**
 * @brief 计算脚的旋转矩阵
 * @details Calculates foot rotation matrix needed for constraint matrix
 *          计算约束矩阵所需的足部旋转矩阵
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
void Constraints::CalculateFootRotationMatrix() {
    R_foot_L << - 1.0*sin(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - cos(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))), -1.0*cos(q(1))*sin(q(0)), cos(q(4))*(cos(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))) - sin(q(4))*(1.0*sin(q(3))*(cos(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(3))*(cos(q(0))*cos(q(2)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)))),
              cos(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) - 1.0*sin(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),      cos(q(0))*cos(q(1)), cos(q(4))*(sin(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) + cos(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))) + sin(q(4))*(cos(q(3))*(cos(q(2))*sin(q(0)) + cos(q(0))*sin(q(1))*sin(q(2))) - 1.0*sin(q(3))*(sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*sin(q(1)))),
                                                                                                                                                                                                                            -1.0*sin(q(2) + q(3) + q(4))*cos(q(1)),              sin(q(1)),                                                                                                                                                                                                                             cos(q(2) + q(3) + q(4))*cos(q(1));
    R_foot_R << - 1.0*sin(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - cos(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))), -1.0*cos(q(6))*sin(q(5)), cos(q(9))*(cos(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + sin(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))) - sin(q(9))*(1.0*sin(q(8))*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) - cos(q(8))*(cos(q(5))*cos(q(7)) - 1.0*sin(q(5))*sin(q(6))*sin(q(7)))),
              cos(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) - 1.0*sin(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),      cos(q(5))*cos(q(6)), cos(q(9))*(sin(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + cos(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))) + sin(q(9))*(cos(q(8))*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) - 1.0*sin(q(8))*(sin(q(5))*sin(q(7)) - 1.0*cos(q(5))*cos(q(7))*sin(q(6)))),
                                                                                                                                                                                                                            -1.0*sin(q(7) + q(8) + q(9))*cos(q(6)),              sin(q(6)),                                                                                                                                                                                                                             cos(q(7) + q(8) + q(9))*cos(q(6));
  

}

/**
 * @brief 获取上界约束
 * @details 
 * @param 
 * @return U_b 上界约束
 * @author wyt
 * @date 2023.11.29
 */
Eigen::VectorXf Constraints::GetUpperBound() const {
    return U_b;
}

/**
 * @brief 获取下界约束
 * @details 
 * @param 
 * @return L_b 下界约束
 * @author wyt
 * @date 2023.11.29
 */
Eigen::VectorXf Constraints::GetLowerBound() const {
    return L_b;
}

/**
 * @brief 获取约束矩阵
 * @details 
 * @param 
 * @return A_c 约束矩阵
 * @author wyt
 * @date 2023.11.29
 */
Eigen::MatrixXf Constraints::GetConstraintMatrix() const {
    return A_c;
}