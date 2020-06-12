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

#include <iostream>

#include "eigen3/Eigen/Dense"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "jacobian/jacobian_autodiff.h"

using std::cout;

DECLARE_int32(v);

struct VectorFunctor1 {
  // T = Value type (e.g. float / double )
  // 2 = Input dimension
  // 4 = Output dimension
  // Input = 2 x 1 matrix.
  // Return = 4 x 1 matrix of result.
  template<class T>
  Eigen::Matrix<T, 4, 1> operator()(const Eigen::Matrix<T, 2, 1>& x) const {
    Eigen::Matrix<T, 4, 1> y;
    y[0] = x[0] * x[0];
    y[1] = x[1] * x[0];
    y[2] = x[1] * x[1];
    y[3] = cos(x[0]) * exp(x[1]);
    return y;
  }
};

TEST(VectorAutoDiff, TwoByFourJacobian) {
  // Define test inputs.
  Eigen::Matrix<double, 2, 1> input;
  input[0] = 10;
  input[1] = -5;

  // Instantiate a VectorFunction object.
  VectorFunctor1 foo;

  // Run autodiff on it.
  auto J = Autodiff<double, 2, 4, VectorFunctor1>(input, foo);

  // Compare result against expected value.
  Eigen::Matrix<double, 4, 2> J_expected;
  J_expected << 2 * input[0],                   0,
                input[1],                       input[0],
                0,                              2 * input[1],
                -sin(input[0]) * exp(input[1]), cos(input[0]) * exp(input[1]);
  if (FLAGS_v > 0) {
    cout << "Autodiff Jacobian:\n" << J << "\n";
    cout << "Analytic Jacobian:\n" << J_expected << "\n";
    cout << "Error:\n" << (J_expected - J).norm() << "\n";
  }
  EXPECT_DOUBLE_EQ((J_expected - J).norm(), 0.0);
}