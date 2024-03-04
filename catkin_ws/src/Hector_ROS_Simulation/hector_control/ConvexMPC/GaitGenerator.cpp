#include "GaitGenerator.h"

// ====================== GAIT Class Implementation ======================= //
/**
 * @brief Constructor: Initializes gait parameters using provided values.
 *        构造函数:使用提供的值初始化步态参数
 * @details 其中调用array将从矩阵运算变为数组运算，支持逐元素一个一个计算
 * @param nMPC_segments MPC片段 <-- horizonLength = 10
 *        用array() 来初始化矩阵
 *        durations 持续时间
 * @return 
 * @author wyt
 * @date 2023.11.29
 */
Gait::Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int> durations, const std::string &name) : _offsets(offsets.array()), 
                                                                                                 _durations(durations.array()),
                                                                                                 _nIterations(nMPC_segments)
{
  // 开辟一个10*2个int类型元素的空间
  _mpc_table = new int[nMPC_segments * 2];
  // cast函数 强制转换int->double 
  // 以行走模式为例 _offsetsPhase = (0,5) / 10 = (0,0.5)
  _offsetsPhase = offsets.cast<double>() / (double)nMPC_segments;    // .cast<double>()转换为double类型
  // 以行走模式为例 _durationsPhase = (5,5) / 10 = (0.5,0.5)
  _durationsPhase = durations.cast<double>() / (double)nMPC_segments; 
  // durations 中的第一个元素，表示支撑阶段的步态步数。 durations 持续时间
  // 以行走模式为例 durations[0]=5
  _stance = durations[0];  
  // 总步数减去支撑阶段的步数，表示摆动阶段的步态步数。
  // 以行走模式为例 nMPC_segments=10 durations=(5,5) _swing=10-5=5; 虽然选择durations[0]，但其实两个值一样
  _swing = nMPC_segments - durations[0]; 
}

/******************************************************************************************************/
/******************************************************************************************************/

Gait::~Gait()
{
  delete[] _mpc_table;
}

/******************************************************************************************************/
/******************************************************************************************************/

// Compute and return the current subphase of contact. 计算并返回支撑的当前子相位
// 相位范围[0,1]
Vec2<double> Gait::getContactSubPhase()
{
  // Array 允许对每个元素进行操作，而不是执行矩阵或向量的矩阵运算。
  // progress 是相对于摆动阶段的当前位置 
  // Array2d: 2x1 的动态double类型数组
  // 以行走模式为例 _offsetsPhase = (0,5) / 10 = (0,0.5)  double   _phase=[0,1]，可以取值范围为1/400的倍数
  Array2d progress = _phase - _offsetsPhase; //所以是(_phase-0,_phase-0.5)



//这一段的作用是把phase分成两部分，
//若程序迭代迭代不到200次返回（迭代次数*2/400,0），
//若程序迭代超过200次不到400次返回（0,迭代次数*2/400）
  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;        
    // 以行走模式为例 _durationsPhase = (5,5) / 10 = (0.5,0.5)
    if (progress[i] > _durationsPhase[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsPhase[i];
    }
  }
  return progress.matrix();
}

/******************************************************************************************************/
/******************************************************************************************************/

// Compute and return the current subphase of swing. 计算并返回摆动的当前子相位。
// 相位范围[0,1]
Vec2<double> Gait::getSwingSubPhase()
{
  // 以行走模式为例 _offsetsPhase=(0,5)/10=(0,0.5)  _durationsPhase=(5,5)/10=(0.5,0.5)
  // 跳跃                       =(0,0)/10=(0,0)                   =(9,9)/10=(0.9,0.9)
  // 踱步                       =(0,9)/10=(0,0.9)                 =(5,5)/10=(0.5,0.5)
  Array2d swing_offset = _offsetsPhase + _durationsPhase; //(0.5,1)
  for (int i = 0; i < 2; i++)
    if (swing_offset[i] > 1)
      // 控制在[0.1]内，只有踱步模式会超，其他模式不会 
      swing_offset[i] -= 1.;
  // 以行走模式为例 _durationsPhase=(5,5)/10=(0.5,0.5) 减去支撑持续时间就是摆动持续时间
  Array2d swing_duration = 1. - _durationsPhase; //(0.5,0)

  Array2d progress = _phase - swing_offset;//_Phase∈（0，1）

  //当迭代次数小于200次时，返回(0,迭代次数*2/400)
  //当迭代次数大于200次小于400次时，返回(迭代次数*2/400，0)
  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

/******************************************************************************************************/
/******************************************************************************************************/

// Generate and return the MPC gait table. 生成并返回一个 MPC步态表
int *Gait::mpc_gait()
{
  for (int i = 0; i < _nIterations; i++)
  {
    // _iteration取值范围为0~9 _nIterations=10
    int iter = (i + _iteration) % _nIterations;        
    // 以行走模式为例_offsets=(0,5)
    Array2i progress = iter - _offsets; // 0 5
    for (int j = 0; j < 2; j++)
    {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 2 + j] = 1;
      else
        _mpc_table[i * 2 + j] = 0;
    }
  }

  return _mpc_table;
}

/******************************************************************************************************/
/******************************************************************************************************/

// Update iteration and phase based on the given values.
// iterationsPerMPC=40 _nIterations=10
void Gait::setIterations(int iterationsPerMPC, int currentIteration)
{
  // iterationsPerMPC=40, 含义是每40次程序迭代，运行一次mpc；
  // currentIteration为当前程序运行的迭代次数。上电后就不断累加
  // _nIterations=10 
  // _iteration是int整数类型 会截取整数部分，例：_iteration=(1/40)%10=0，每40次才加1，且取值范围为0~9
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  //_phase取值为0-1，每0.4秒（400次迭代）完成一次phase。phase为0，表示脚起步，为1表示脚落地。
  // 例 _phase=(1%(40*10))/(40*10)=(1/400) 即将_phase限制在400次内 且归一化
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double)(iterationsPerMPC * _nIterations);

}