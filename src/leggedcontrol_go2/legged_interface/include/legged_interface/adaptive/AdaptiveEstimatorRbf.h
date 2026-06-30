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
  scalar_t rbfWidth{1.0}; // RBF kernel width (b_j)

  // ── Adaptation gains (fixed for Phase 1, variable for Phase 3) ────
  scalar_t learningRateForce{0.5};   // Force channel learning rate
  scalar_t learningRateTorque{0.1};  // Torque channel learning rate

  // ── Weight regularization ──────────────────────────────────────────
  scalar_t weightDecay{1e-4};  // L2 regularization on weights

  // ── Clamping ───────────────────────────────────────────────────────
  scalar_t maxForceEstimate{200.0};   // N
  scalar_t maxTorqueEstimate{50.0};   // Nm

  // ── State space bounds for center initialization ───────────────────
  vector_t chiMin;   // Lower bound for each input dimension
  vector_t chiMax;   // Upper bound for each input dimension
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

  void update(const vector_t& state, const vector_t& stateDes,
              scalar_t dt) override;

  EstimatorOutput getOutput() const override;
  void reset() override;
  std::string getName() const override { return "RBF (Paper B — Dual-RBFNN)"; }

  /// Get the current 6D wrench estimate (for CLF constraint in MPC)
  vector6_t getWrenchEstimate() const { return wrenchEstimate_; }

  /// Get RBFNN config
  const RbfEstimatorConfig& getRbfConfig() const { return rbfConfig_; }

 private:
  /**
   * Build input feature vector chi from state.
   */
  vector_t buildInputFeatures(const vector_t& state,
                               const vector_t& stateDes) const;

  /**
   * Forward pass: chi -> wrench estimate.
   */
  vector6_t forward(const vector_t& chi) const;

  /**
   * Compute RBF activations h(chi).
   */
  vector_t computeActivations(const vector_t& chi) const;

  /**
   * Update RBFNN weights using gradient descent.
   */
  void updateWeights(const vector6_t& sigma,
                     const vector_t& activations, scalar_t dt);

  RbfEstimatorConfig rbfConfig_;

  // ── RBFNN parameters ───────────────────────────────────────────────
  matrix_t centers_;     // [inputDim x nCenters] RBF center vectors
  vector_t widths_;      // [nCenters] kernel widths
  matrix_t weights_;     // [6 x nCenters] output weights
  vector6_t wrenchEstimate_;  // Latest wrench output

  // ── Internal state ─────────────────────────────────────────────────
  EstimatorOutput lastOutput_;
};

}  // namespace adaptive
}  // namespace legged
