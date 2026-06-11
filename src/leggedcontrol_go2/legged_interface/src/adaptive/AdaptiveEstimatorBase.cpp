/******************************************************************************
 * AdaptiveEstimatorBase.cpp
 *
 * Shared implementation for the abstract adaptive estimator interface.
 ******************************************************************************/

#include "legged_interface/adaptive/AdaptiveEstimatorBase.h"

#include <angles/angles.h>

namespace legged {
namespace adaptive {

ocs2::vector6_t AdaptiveEstimatorBase::computeCompositeError(
    const ocs2::vector_t& state, const ocs2::vector_t& stateDes) const {

  // State layout: [v_com(3), L/m(3), p(3), eulerZYX(3)]
  const ocs2::vector3_t v_current = state.segment<3>(0);
  const ocs2::vector3_t v_desired = stateDes.segment<3>(0);
  const ocs2::vector3_t v_tilde = v_current - v_desired;

  const ocs2::vector3_t p_current = state.segment<3>(6);
  const ocs2::vector3_t p_desired = stateDes.segment<3>(6);
  const ocs2::vector3_t p_tilde = p_current - p_desired;

  // Angular velocity from normalized angular momentum
  // L/m is stored in state[3:5], actual omega depends on inertia
  // For simplicity, use state[3:5] directly as omega approximation
  const ocs2::vector3_t omega_current = state.segment<3>(3);
  const ocs2::vector3_t omega_desired = stateDes.segment<3>(3);
  const ocs2::vector3_t omega_tilde = omega_current - omega_desired;

  // Euler angle error (with yaw wrapping)
  ocs2::vector3_t euler_current = state.segment<3>(9);
  ocs2::vector3_t euler_desired = stateDes.segment<3>(9);
  euler_current(0) = euler_desired(0) +
      angles::shortest_angular_distance(euler_desired(0), euler_current(0));
  const ocs2::vector3_t euler_tilde = euler_current - euler_desired;

  // Composite error: sigma = v_tilde + Lambda * position_tilde
  ocs2::vector6_t sigma;
  sigma.head<3>() = v_tilde + config_.Lambda_l * p_tilde;
  sigma.tail<3>() = omega_tilde + config_.Lambda_o * euler_tilde;

  return sigma;
}

}  // namespace adaptive
}  // namespace legged
