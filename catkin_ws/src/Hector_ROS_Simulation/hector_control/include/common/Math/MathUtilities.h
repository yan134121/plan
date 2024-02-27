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

/*! @file MathUtilities.h
 *  @brief Utility functions for math
 *
 */

#ifndef PROJECT_MATHUTILITIES_H
#define PROJECT_MATHUTILITIES_H

#include <eigen3/Eigen/Dense>

/*!
 * Square a number 求平方
 */
template <typename T>
T square(T a) {
  return a * a;
}

/*!
 * Are two eigen matrices almost equal? 
   这是一个用于检查两个 Eigen 矩阵是否在给定的误差范围内近似相等的函数。
   函数的目的是遍历矩阵中的每个元素，计算其差的绝对值，并检查是否小于给定的容忍度（tol）。
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b,
                 T2 tol) {
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;
  // Eigen::Dynamic用于表示动态大小，而RowsAtCompileTime和ColsAtCompileTime表示在编译时确定的常量。
  if (T::RowsAtCompileTime == Eigen::Dynamic ||
      T::ColsAtCompileTime == Eigen::Dynamic) {
    /*
    assert 是一个宏，用于在程序运行时进行调试时的断言检查。
    在 C++ 中，assert 宏用于检查一个表达式是否为真。
    如果表达式的结果为假（即为 0），则 assert 宏将触发一个错误，
    打印一条错误消息，并终止程序的执行。
    判断两个矩阵的行数列数是否一致
    */
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol) return false;
    }
  }
  return true;
}

#endif  // PROJECT_MATHUTILITIES_H
