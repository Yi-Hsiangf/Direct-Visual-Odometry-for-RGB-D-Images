/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <sophus/se3.hpp>

#include <visnav/common_types.h>

#define bias std::numeric_limits<T>::epsilon()

namespace visnav {

// Implement exp for SO(3)
template <class T>
Eigen::Matrix<T, 3, 3> user_implemented_expmap(
    const Eigen::Matrix<T, 3, 1>& xi) {
  // TODO SHEET 1: implement
  double theta = xi.norm();
  Eigen::Matrix<T, 3, 3> xi_cap;
  Eigen::Matrix<T, 3, 3> R;

  xi_cap << 0, -xi(2), xi(1), xi(2), 0, -xi(0), -xi(1), xi(0), 0;

  R = Eigen::MatrixXd::Identity(3, 3) +
      xi_cap * (sin(theta) + bias) / (theta + bias) +
      xi_cap * xi_cap * (1 + 0.5 * bias - cos(theta)) / (theta * theta + bias);
  return R;
}

// Implement log for SO(3)
template <class T>
Eigen::Matrix<T, 3, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 3, 3>& mat) {
  // TODO SHEET 1: implement
  double theta = acos((mat.trace() - 1) / 2);
  Eigen::Matrix<T, 3, 1> w;

  w(0) = mat(2, 1) - mat(1, 2);
  w(1) = mat(0, 2) - mat(2, 0);
  w(2) = mat(1, 0) - mat(0, 1);
  w *= (theta + bias) / (2 * sin(theta) + 2 * bias);
  return w;
}

// Implement exp for SE(3)
template <class T>
Eigen::Matrix<T, 4, 4> user_implemented_expmap(
    const Eigen::Matrix<T, 6, 1>& xi) {
  // TODO SHEET 1: implement
  Eigen::Matrix<T, 4, 4> Trans;
  Eigen::Matrix<T, 3, 1> w, v;
  Eigen::Matrix<T, 3, 3> J, R, w_cap;
  double theta;

  v = xi.head(3);
  w = xi.tail(3);
  w_cap << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  R = user_implemented_expmap(w);
  theta = sqrt(w.transpose() * w);

  J = Eigen::MatrixXd::Identity(3, 3) +
      w_cap * (1 - cos(theta) + 0.5 * bias) / (pow(theta, 2) + bias) +
      w_cap * w_cap * (theta + bias - sin(theta)) / (pow(theta, 3) + 6 * bias);

  Trans.block(0, 0, 3, 3) = R;
  Trans.block(0, 3, 3, 1) = J * v;
  Trans.block(3, 0, 1, 3) = Eigen::MatrixXd::Zero(1, 3);
  Trans(3, 3) = 1;

  return Trans;
}

// Implement log for SE(3)
template <class T>
Eigen::Matrix<T, 6, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 4, 4>& mat) {
  // TODO SHEET 1: implement
  Eigen::Matrix<T, 6, 1> xi;
  Eigen::Matrix<T, 3, 1> w, t;
  Eigen::Matrix<T, 3, 3> J, R, w_cap;
  double theta, A;

  R = mat.block(0, 0, 3, 3);
  t = mat.block(0, 3, 3, 1);
  w = user_implemented_logmap(R);
  w_cap << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  theta = w.norm();

  A = (2 * sin(theta) - theta - theta * cos(theta) + bias) /
      (2 * pow(theta, 2) * sin(theta) + 12 * bias);
  J = Eigen::MatrixXd::Identity(3, 3) - 0.5 * w_cap + A * w_cap * w_cap;
  xi.block(0, 0, 3, 1) = J * t;
  xi.block(3, 0, 3, 1) = w;
  return xi;
}

}  // namespace visnav
