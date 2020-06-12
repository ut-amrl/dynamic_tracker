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

#include "jacobian/jacobian_autodiff.h"

#ifndef SRC_EKF_TRACKER_EKF_TRACKER_H_
#define SRC_EKF_TRACKER_EKF_TRACKER_H_

namespace ekf {

static const bool kDebug = true;

// A passive EKF tracker (i.e. no controls input).
template <int kStateSize, int kObservationSize,
    typename MotionModel, typename ObservationModel>
class EkfTracker {
 typedef Eigen::Matrix<float, kStateSize, 1> State;
 typedef Eigen::Matrix<float, kStateSize, kStateSize> Covariance;
 typedef Eigen::Matrix<float, kObservationSize, 1> Observation;

 public:
  // Default constructor: do nothing, just initialize pointers.
  EkfTracker() : obs_model_(nullptr), motion_model_(nullptr) {}
  // Set motion model and observation model functors.
  void SetModels(MotionModel* motion_model, ObservationModel* obs_model) {
    motion_model_ = motion_model;
    obs_model_ = obs_model;
  }

  // Initialize the tracker state and time.
  void Initialize(const State& x, const Covariance& p, const double t) {
    x_ = x;
    p_ = p;
    time_ = t;
  }
  // Predict the state forward by delta-time dt.
  void Predict(double dt) {
    auto h = *motion_model_;
    const auto f_jacobian =
        jacobian::Autodiff<float, kStateSize, kStateSize, MotionModel>(
            x_, h, dt);
    const auto process_noise = h.ProcessNoise(x_, dt);
    if (kDebug) {
      std::cout << "\nPredict\npre: x = \n" << x_ << "\n"
                << "p = \n" << p_ << "\n"
                << "F = \n" << f_jacobian << "\n";

    }
    x_ = h(x_, dt);
    p_ = f_jacobian * p_ * f_jacobian.transpose() + process_noise;
    if (kDebug) {
      std::cout << "post: x = \n" << x_ << "\np =\n" << p_ << "\n";
    }
    time_ += dt;
  }
  // Apply an observation update.
  void Update(const Observation& s, const double t) {
    // Ignore observations older than current time.
    if (t < time_) return;
    if (t > time_) {
      Predict(t - time_);
    }
    auto h = *obs_model_;
    const auto h_jacobian =
        jacobian::Autodiff<float, kStateSize, kObservationSize,
            ObservationModel>(x_, h, t);
    if (kDebug) {
      std::cout << "\nUpdate\npre: x = \n" << x_ << "\n"
                << "p = \n" << p_ << "\n"
                << "H = \n" << h_jacobian << "\n"
                << "s = \n" << s << "\n";
    }
    const auto observation_noise = h.ObservationNoise(x_, t);
    const auto innovation = s - h(x_, t);
    const auto innovation_cov =
        h_jacobian * p_ * h_jacobian.transpose() + observation_noise;
    const auto kalman_gain =
        p_ * h_jacobian.transpose() * innovation_cov.inverse();
    x_ = x_ + kalman_gain * innovation;
    p_ = (Covariance::Identity() - kalman_gain * h_jacobian) * p_;
    time_ = t;
    if (kDebug) {
      std::cout << "innovation = \n" << innovation << "\n"
                << "innovation_cov = \n" << innovation_cov << "\n"
                << "kalman_gain = \n" << kalman_gain << "\n";
      std::cout << "post: x = \n" << x_ << "\np = \n" << p_ << "\n";
    }
  }
  State GetState() const {
    return x_;
  }
  Covariance GetCovariance() const {
    return p_;
  }

 private:
  // Current state estimate, from
  State x_;
  Covariance p_;
  double time_;
  ObservationModel* obs_model_;
  MotionModel* motion_model_;
};

}  // namespace ekf_tracker

#endif  // SRC_EKF_TRACKER_EKF_TRACKER_H_
