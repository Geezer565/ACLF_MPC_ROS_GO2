/******************************************************************************
 * AdaptiveEstimatorRbf.h
 *
 * Dual-RBFNN-based adaptive estimator (Paper B — VAN-MPC style).
 *
 * Replaces the physics-based 16-param regressor Yu with a Dual-RBF neural
 * network that directly outputs 6D adaptive wrench [f_u; t_u].
 *
 * Key innovations from Paper B (Liu et al., 2025):
 *   1. Dual-RBFNN: two sub-networks (force + torque), learned online
 *   2. Composite error: E_c = gamma * E_e + (1-gamma) * sigma
 *      (Phase 2 — currently uses sigma only, gamma=0)
 *   3. Variable step-size: Gamma adapts based on ||chi - c_j||
 *      (Phase 3 — currently fixed Gamma)
 *
 * RBFNN structure:
 *   Input  chi in R^{inputDim}   — state features
 *   Hidden h_j = exp(-||chi - c_j||^2 / b_j^2), j=1..nCenters
 *   Output y = W * h(chi) in R^6  — [f_x, f_y, f_z, t_x, t_y, t_z]
 *
 * Weight update (gradient descent on sigma):
 *   dW/dt = -Gamma * sigma * h(chi)^T
 *   => W += -Gamma * sigma * h(chi)^T * dt  (discrete)
 ******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>

#include <vector>

#include "legged_interface/adaptive/AdaptiveEstimatorBase.h"

namespace legged {
namespace adaptive {

/**
 * RBF estimator configuration.
 */
struct RbfEstimatorConfig : public EstimatorConfig {
  // ── RBFNN architecture ─────────────────────────────────────────────
  int nCenters{21};            // Number of RBF centers (2m+1, m=10)
  int inputDim{12};            // Input feature dimension
  ocs2::scalar_t rbfWidth{1.0}; // RBF kernel width (b_j)

  // ── Adaptation gains (fixed for Phase 1, variable for Phase 3) ────
  ocs2::scalar_t learningRateForce{0.5};   // Force channel learning rate
  ocs2::scalar_t learningRateTorque{0.1};  // Torque channel learning rate

  // ── Weight regularization ──────────────────────────────────────────
  ocs2::scalar_t weightDecay{1e-4};  // L2 regularization on weights

  // ── Clamping ───────────────────────────────────────────────────────
  ocs2::scalar_t maxForceEstimate{200.0};   // N
  ocs2::scalar_t maxTorqueEstimate{50.0};   // Nm

  // ── State space bounds for center initialization ───────────────────
  ocs2::vector_t chiMin;   // Lower bound for each input dimension
  ocs2::vector_t chiMax;   // Upper bound for each input dimension
};

/**
 * Dual-RBFNN adaptive estimator.
 *
 * Directly learns a 6D wrench compensation from state features,
 * no physics-based regressor matrix required.
 *
 * Phase 1 (current): fixed Gamma, sigma-driven update
 * Phase 2 (future):  composite error E_c
 * Phase 3 (future):  variable step-size
 */
class AdaptiveEstimatorRbf : public AdaptiveEstimatorBase {
 public:
  explicit AdaptiveEstimatorRbf(const RbfEstimatorConfig& config);
  ~AdaptiveEstimatorRbf() override = default;

  void update(const ocs2::vector_t& state, const ocs2::vector_t& stateDes,
              ocs2::scalar_t dt) override;

  EstimatorOutput getOutput() const override;
  void reset() override;
  std::string getName() const override { return "RBF (Paper B — Dual-RBFNN)"; }

  /// Get the current 6D wrench estimate (for CLF constraint in MPC)
  ocs2::vector6_t getWrenchEstimate() const { return wrenchEstimate_; }

  /// Get RBFNN config
  const RbfEstimatorConfig& getRbfConfig() const { return rbfConfig_; }

 private:
  /**
   * Build input feature vector chi from state.
   *
   * chi includes:
   *   [0-2]   position error p_tilde
   *   [3-5]   orientation error (Euler angles)
   *   [6-8]   linear velocity error v_tilde
   *   [9-11]  angular velocity error omega_tilde
   *
   * Normalized to [-1, 1] range using chiMin/chiMax.
   */
  ocs2::vector_t buildInputFeatures(const ocs2::vector_t& state,
                                     const ocs2::vector_t& stateDes) const;

  /**
   * Forward pass: chi -> wrench estimate.
   * y = W * h(chi) where h_j = exp(-||chi - c_j||^2 / b_j^2)
   */
  ocs2::vector6_t forward(const ocs2::vector_t& chi) const;

  /**
   * Compute RBF activations h(chi).
   */
  ocs2::vector_t computeActivations(const ocs2::vector_t& chi) const;

  /**
   * Update RBFNN weights using gradient descent.
   * dW/dt = -gamma * sigma * h^T - weightDecay * W
   */
  void updateWeights(const ocs2::vector6_t& sigma,
                     const ocs2::vector_t& activations, ocs2::scalar_t dt);

  RbfEstimatorConfig rbfConfig_;

  // ── RBFNN parameters ───────────────────────────────────────────────
  ocs2::matrix_t centers_;     // [inputDim x nCenters] RBF center vectors
  ocs2::vector_t widths_;      // [nCenters] kernel widths
  ocs2::matrix_t weights_;     // [6 x nCenters] output weights
  ocs2::vector6_t wrenchEstimate_;  // Latest wrench output

  // ── Internal state ─────────────────────────────────────────────────
  EstimatorOutput lastOutput_;
};

}  // namespace adaptive
}  // namespace legged
