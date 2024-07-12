/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of this work nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <perceptive_mpc/SystemDynamics.h>
#include <iostream>

// #define USE_MABI

using namespace perceptive_mpc;

#ifdef USE_MABI
void SystemDynamics::systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                   ad_dynamic_vector_t& stateDerivative) const {

  Eigen::Quaternion<ad_scalar_t> currentRotation(state.head<Definitions::BASE_STATE_DIM_>().head<4>());
  // std::cout << "[SystemDynamics::systemFlowMap]" << std::endl;
  // auto coeffs = currentRotation.coeffs();  // 获取四元数的系数
  // std::cout << "Quaternion (x, y, z, w): ";
  // for (int i = 0; i < 4; ++i) {
  //     // std::cout << static_cast<double>(coeffs[i]) << " ";
  //     std::cout << coeffs.coeff(i) << " ";  // 使用coeff()方法获取系数值
  // }
  // std::cout << std::endl;


  // Eigen::Quaternion<ad_scalar_t> normalizedRotation = currentRotation.normalized();

// #ifdef USE_MABI                              
  ad_scalar_t linearVelocity = input.head<Definitions::BASE_INPUT_DIM_>()[0];
  ad_scalar_t omega = input.head<Definitions::BASE_INPUT_DIM_>()[1];

  // Eigen::Quaternion<ad_scalar_t> currentRotation(state.head<Definitions::BASE_STATE_DIM_>().head<4>());

  // derivative of orientation quaternion: https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
  Eigen::Quaternion<ad_scalar_t> deltaRotation;
  deltaRotation.w() = 0;
  deltaRotation.x() = 0;
  deltaRotation.y() = 0;
  deltaRotation.z() = omega / 2;
  stateDerivative.head<Definitions ::BASE_STATE_DIM_>().head<4>() = (currentRotation * deltaRotation).coeffs();

  Eigen::Matrix<ad_scalar_t, 3, 1> linearVelocityVector = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
  linearVelocityVector[0] = linearVelocity;
  stateDerivative.head<Definitions ::BASE_STATE_DIM_>().tail<3>() = currentRotation * linearVelocityVector;
  stateDerivative.head<Definitions ::BASE_STATE_DIM_>().tail<3>()[2] = 0;

// #else
//   // 底盘的角速度和线速度导数设置为零
//   Eigen::Quaternion<ad_scalar_t> zeroDeltaRotation;
//   zeroDeltaRotation.w() = 1; // 单位四元数表示无旋转
//   zeroDeltaRotation.x() = 0;
//   zeroDeltaRotation.y() = 0;
//   zeroDeltaRotation.z() = 0;
//   stateDerivative.head<Definitions::BASE_STATE_DIM_>().head<4>() = (normalizedRotation * zeroDeltaRotation).coeffs();
//   // 底盘的位置导数设置为零向量
//   Eigen::Matrix<ad_scalar_t, 3, 1> zeroLinearVelocityVector = Eigen::Matrix<ad_scalar_t, 3, 1>::Zero();
//   stateDerivative.head<Definitions::BASE_STATE_DIM_>().tail<3>() = zeroLinearVelocityVector;
//   stateDerivative.head<Definitions ::BASE_STATE_DIM_>().tail<3>()[2] = 0;
// #endif

  // Arm 的stateDerivative
  stateDerivative.tail<Definitions ::ARM_STATE_DIM_>() = input.tail<Definitions::ARM_STATE_DIM_>();
}

#else
void SystemDynamics::systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                   ad_dynamic_vector_t& stateDerivative) const {
  stateDerivative.tail<Definitions ::ARM_STATE_DIM_>() = input.tail<Definitions::ARM_STATE_DIM_>();
                                   } 
#endif                        

SystemDynamics* SystemDynamics::clone() const { return new SystemDynamics(*this); }
