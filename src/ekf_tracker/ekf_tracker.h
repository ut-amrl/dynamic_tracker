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

#include "eigen3/Eigen/Dense"

#ifndef SRC_EKF_TRACKER_EKF_TRACKER_H_
#define SRC_EKF_TRACKER_EKF_TRACKER_H_

namespace ekf_tracker {

template <typename T, int kStateSize, int kObservationSize>
class EkfTracker {
 typedef Eigen::Matrix<T, kStateSize, 1> State;
 typedef Eigen::Matrix<T, kStateSize, kStateSize> Covariance;
 typedef Eigen::Matrix<T, kObservationSize, 1> Observation;

 public:
  void Initialize(const State& x);
  void Predict();
  void Update(const Observation& s);
  State GetState() const;
  Covariance GetCovariance() const;

 private:
  State x_;
};

}  // namespace ekf_tracker

#endif  // SRC_EKF_TRACKER_EKF_TRACKER_H_
