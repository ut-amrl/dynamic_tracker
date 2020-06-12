// Copyright (C) 2020 Joydeep Biswas
// University of Texas at Austin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================

#ifndef JACOBIAN_AUTODIFF_H
#define JACOBIAN_AUTODIFF_H

#include "ceres/jet.h"
#include "eigen3/Eigen/Dense"

namespace jacobian {

// T = basic type (float / double)
// M = Input dimension
// N = Output dimension
// F = the function being differentiated.
//     Takes in an M x 1 vector, returns an N x 1 vector.
// F can optionally also take in a variable number of additional parameters.
// Return = N x M matrix of Jacobian.
template<typename T, int M, int N, typename F, class ...AdditionalArgs>
Eigen::Matrix<T, N, M> Autodiff(
    const Eigen::Matrix<T, M, 1>& x,
    const F& f,
    AdditionalArgs... additional_args) {
  // Allocate the return Jacobian matrix.
  Eigen::Matrix<T, N, M> J;

  Eigen::Matrix<ceres::Jet<T, M>, M, 1> input;
  // Set all inputs to the provided values.
  for (int i = 0; i < M; ++i) {
    // Set value of input i.
    input[i].a = x[i];
    // The differentials of all inputs are 0 wrt input i ...
    input[i].v.setZero();
    // ... except input i, and the differential of input i wrt input i is 1.
    input[i].v[i] = 1;
  }

  // Call the function with this input, and store the result.
  const Eigen::Matrix<ceres::Jet<T, M>, N, 1> output =
      f(input, additional_args...);

  // For all output dimensions j.
  for (int j = 0; j < N; ++j) {
    for (int i = 0; i < M; ++i) {
      // Set output Jacobian J(j, i)
      J(j, i) = output[j].v[i];
    }
  }

  // Return the resultant Jacobian.
  return J;
}

}  // namespace jacobian

#endif  // JACOBIAN_AUTODIFF_H