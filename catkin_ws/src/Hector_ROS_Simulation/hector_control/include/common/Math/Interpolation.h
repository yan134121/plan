/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>
/*
  这是一个 C++ 命名空间 Interpolate 中的线性插值函数模板。该模板提供了一个线性插值函数 lerp，
  用于在给定范围内在两个值之间进行插值。具体来说，它接受两个值 y0 和 yf，以及一个插值参数 x，
  并返回在这两个值之间进行线性插值的结果。这里的 y_t 和 x_t 是模板参数，分别表示值的类型和插值参数的类型。
  要求 x_t 必须是浮点数类型，这通过 static_assert 进行了检查。函数确保插值参数 x 处于 0 和 1 之间。
*/
namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
   y0和yf之间的线性插值。X在0和1之间
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  /*
    assert 是一个宏，用于在程序运行时进行调试时的断言检查。
    在 C++ 中，assert 宏用于检查一个表达式是否为真。
    如果表达式的结果为假（即为 0），则 assert 宏将触发一个错误，
    打印一条错误消息，并终止程序的执行。*/
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
   在y0和yf之间创建一个新的一阶滤波器三次bezier插值。X在0和1之间
   不是标准的三阶贝塞尔曲线，而是三次方贝塞尔，
   x*x*x 表示 x 的三次方，这是三次方程的基本部分
   x_t(3) * (x * x * (x_t(1) - x)) 是贝塞尔曲线的控制点系数
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  // 取值在[0,1] bezier = x^3 + 3*[x^2*(1-x)]
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
   三次方贝塞尔曲线的一阶导数
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  // bezier = x^3 + 3*[x^2*(1-x)]求导 = 6*x - 6*x^2
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
   三次方贝塞尔曲线的二阶导数
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  /*
    assert 是一个宏，用于在程序运行时进行调试时的断言检查。
    在 C++ 中，assert 宏用于检查一个表达式是否为真。
    如果表达式的结果为假（即为 0），则 assert 宏将触发一个错误，
    打印一条错误消息，并终止程序的执行。*/
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = - x_t(12) * x;
  return bezier * yDiff;
}

}  // namespace Interpolate

#endif  // PROJECT_INTERPOLATION_H
