/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "../../include/common/Math/Interpolation.h"
#include "../../include/common/FootSwingTrajectory.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1) 现在所处的摆动相位 
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T> //T 是一个占位符 模板 可以是任意类型
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) 
{
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase);

  T zp, zv;

  // 对z轴的位置和速度重新赋值，因为三次方只能算一半的轨迹，即从最高点分成两份
  if(phase < T(0.5)) 
  {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2);
  } else 
  {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
  }
  // 计算出来z轴重新覆盖
  _p[2] = zp;
  _v[2] = zv;
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;