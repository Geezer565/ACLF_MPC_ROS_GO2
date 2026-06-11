/******************************************************************************
 * AdaptiveEstimatorLegacy.h
 *
 * Wraps the original Paper A ACLF-MPC estimator in the strategy-pattern
 * interface. Internally uses:
 *   - AdaptiveParams (16-dim physical parameters)
 *   - AdaptiveRegressor (6x16 Yu matrix)
 *   - Slotine-Li update law: pi_hat += Gamma * Yu^T * sigma * dt
 *
 * This class does NOT modify the original code — it delegates to it.
 ******************************************************************************/

#pragma once

#include <memory>

#include "legged_interface/adaptive/AdaptiveEstimatorBase.h"

// Forward declarations from ocs2_legged_robot
namespace ocs2 {
namespace legged_robot {
namespace adaptive {
struct AdaptiveParams;
}  // namespace adaptive
}  // namespace legged_robot
}  // namespace ocs2

namespace legged {
namespace adaptive {

/**
 * Legacy estimator configuration (extends base config).
 */
struct LegacyEstimatorConfig : public EstimatorConfig {
  // Adaptation gains (diagonal of 16x16 Gamma matrix)
  ocs2::scalar_t gammaMass{5.0};       // Payload mass adaptation gain
  ocs2::scalar_t gammaCom{1.0};        // CoM offset adaptation gain
  ocs2::scalar_t gammaInertia{0.01};   // Inertia adaptation gain
  ocs2::scalar_t gammaWrench{0.1};     // Constant wrench adaptation gain
};

/**
 * Legacy adaptive estimator: 16-parameter Slotine-Li regressor.
 *
 * Implements Paper A (Minniti et al., 2021):
 *   pi_hat in R^16  = [m_u(1), h_u(3), vec(I_u)(6), f_const(3), t_const(3)]
 *   Yu in R^{6x16}  = force/torque regressor matrix
 *   pi_dot = Gamma * Yu^T * sigma
 *
 * The adaptive wrench [f_u; t_u] is computed from pi_hat via:
 *   f_u = m_u * (v_dot_pr - g) + f_const
 *   t_u = I_u * omega_dot_r + omega x (I_u * omega) + t_const
 */
class AdaptiveEstimatorLegacy : public AdaptiveEstimatorBase {
 public:
  explicit AdaptiveEstimatorLegacy(const LegacyEstimatorConfig& config);
  ~AdaptiveEstimatorLegacy() override;

  void update(const ocs2::vector_t& state, const ocs2::vector_t& stateDes,
              ocs2::scalar_t dt) override;

  EstimatorOutput getOutput() const override;
  void reset() override;
  std::string getName() const override { return "Legacy (Paper A — 16-param Slotine-Li)"; }

  /// Access the underlying AdaptiveParams (for CLF constraint)
  ocs2::legged_robot::adaptive::AdaptiveParams* getAdaptiveParamsPtr() {
    return paramsPtr_.get();
  }

  /// Get config
  const LegacyEstimatorConfig& getLegacyConfig() const { return legacyConfig_; }

 private:
  LegacyEstimatorConfig legacyConfig_;
  std::unique_ptr<ocs2::legged_robot::adaptive::AdaptiveParams> paramsPtr_;
  EstimatorOutput lastOutput_;
};

}  // namespace adaptive
}  // namespace legged
