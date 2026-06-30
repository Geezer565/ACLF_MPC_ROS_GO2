/******************************************************************************
 * AdaptiveEstimatorRbf.cpp
 *
 * Implementation of the Dual-RBFNN adaptive estimator (Paper B style).
 ******************************************************************************/

#include "legged_interface/adaptive/AdaptiveEstimatorRbf.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace legged {
namespace adaptive {

// ── Constructor ───────────────────────────────────────────────────────────
AdaptiveEstimatorRbf::AdaptiveEstimatorRbf(const RbfEstimatorConfig& config)
    : AdaptiveEstimatorBase(config), rbfConfig_(config) {
  const int D = config.inputDim;   // Input dimension
  const int N = config.nCenters;   // Number of RBF centers (2m+1)

  // Allocate RBFNN parameters
  centers_.resize(D, N);
  widths_.resize(N);
  weights_.resize(6, N);

  // Initialize widths
  widths_.setConstant(config.rbfWidth);

  // Initialize centers: uniform grid spanning chiMin to chiMax
  // For dimensions without explicit bounds, use default [-1, 1]
  if (config.chiMin.size() == D && config.chiMax.size() == D) {
    for (int d = 0; d < D; ++d) {
      const ocs2::scalar_t lo = config.chiMin(d);
      const ocs2::scalar_t hi = config.chiMax(d);
      const ocs2::scalar_t step = (N > 1) ? (hi - lo) / (N - 1) : 0.0;
      for (int j = 0; j < N; ++j) {
        centers_(d, j) = lo + step * j;
      }
    }
  } else {
    // Default: centers spread across [-1, 1] for each input dimension
    for (int d = 0; d < D; ++d) {
      const ocs2::scalar_t step = (N > 1) ? 2.0 / (N - 1) : 0.0;
      for (int j = 0; j < N; ++j) {
        centers_(d, j) = -1.0 + step * j;
      }
    }
  }

  reset();
}

// ── Reset ──────────────────────────────────────────────────────────────────
void AdaptiveEstimatorRbf::reset() {
  weights_.setZero();
  wrenchEstimate_.setZero();
  lastOutput_ = EstimatorOutput{};
}

// ── Build Input Features ───────────────────────────────────────────────────
ocs2::vector_t AdaptiveEstimatorRbf::buildInputFeatures(
    const ocs2::vector_t& state, const ocs2::vector_t& stateDes) const {

  // State layout: [v_com(3), L/m(3), p(3), euler(3)]
  // Reference has same layout

  ocs2::vector_t chi(rbfConfig_.inputDim);
  chi.setZero();

  // Position error (3D) — normalized
  chi.segment<3>(0) = state.segment<3>(6) - stateDes.segment<3>(6);

  // Orientation error (3D) — Euler angle difference, wrap to [-pi, pi]
  for (int i = 0; i < 3; ++i) {
    chi(3 + i) = state(9 + i) - stateDes(9 + i);
    // Wrap to [-pi, pi]
    while (chi(3 + i) > M_PI) chi(3 + i) -= 2.0 * M_PI;
    while (chi(3 + i) < -M_PI) chi(3 + i) += 2.0 * M_PI;
  }

  // Linear velocity error (3D)
  chi.segment<3>(6) = state.segment<3>(0) - stateDes.segment<3>(0);

  // Angular velocity error (3D) — from normalized angular momentum
  chi.segment<3>(9) = state.segment<3>(3) - stateDes.segment<3>(3);

  // Normalize to [-1, 1] range using configured bounds
  if (rbfConfig_.chiMin.size() == rbfConfig_.inputDim &&
      rbfConfig_.chiMax.size() == rbfConfig_.inputDim) {
    for (int i = 0; i < rbfConfig_.inputDim; ++i) {
      const ocs2::scalar_t lo = rbfConfig_.chiMin(i);
      const ocs2::scalar_t hi = rbfConfig_.chiMax(i);
      const ocs2::scalar_t range = (hi - lo) > 1e-6 ? (hi - lo) : 1.0;
      chi(i) = 2.0 * (chi(i) - lo) / range - 1.0;  // Map to [-1, 1]
      chi(i) = std::max(-1.0, std::min(1.0, chi(i)));  // Clamp
    }
  }

  return chi;
}

// ── Compute RBF Activations ────────────────────────────────────────────────
ocs2::vector_t AdaptiveEstimatorRbf::computeActivations(
    const ocs2::vector_t& chi) const {

  const int N = rbfConfig_.nCenters;
  ocs2::vector_t h(N);

  for (int j = 0; j < N; ++j) {
    const ocs2::vector_t diff = chi - centers_.col(j);
    const ocs2::scalar_t sqDist = diff.squaredNorm();
    const ocs2::scalar_t w = widths_(j);
    h(j) = std::exp(-sqDist / (w * w));
  }

  return h;
}

// ── Forward Pass ───────────────────────────────────────────────────────────
vector6_t AdaptiveEstimatorRbf::forward(
    const vector_t& chi) const {

  const vector_t h = computeActivations(chi);

  // y = W * h  (6 x nCenters) * (nCenters x 1) = (6 x 1)
  vector6_t y = weights_ * h;

  return y;
}

// ── Update Weights ─────────────────────────────────────────────────────────
void AdaptiveEstimatorRbf::updateWeights(const vector6_t& sigma,
                                          const ocs2::vector_t& activations,
                                          ocs2::scalar_t dt) {
  if (dt <= 0.0) return;

  // dW/dt = -Gamma * sigma * h^T  -  weightDecay * W
  //
  // This is gradient descent on the error: minimizing sigma^T * sigma
  // where sigma is the sliding surface. The RBFNN output enters the
  // dynamics as a wrench compensation, so reducing sigma indirectly
  // drives the RBFNN weights toward the true uncertainty.

  // Separate learning rates for force (rows 0-2) and torque (rows 3-5)
  for (int row = 0; row < 3; ++row) {
    // Force channel
    const ocs2::scalar_t lr = rbfConfig_.learningRateForce;
    for (int j = 0; j < rbfConfig_.nCenters; ++j) {
      // Gradient step
      weights_(row, j) -= lr * sigma(row) * activations(j) * dt;

      // Weight decay (L2 regularization)
      weights_(row, j) -= lr * rbfConfig_.weightDecay * weights_(row, j) * dt;
    }
  }

  for (int row = 3; row < 6; ++row) {
    // Torque channel
    const ocs2::scalar_t lr = rbfConfig_.learningRateTorque;
    for (int j = 0; j < rbfConfig_.nCenters; ++j) {
      weights_(row, j) -= lr * sigma(row) * activations(j) * dt;
      weights_(row, j) -= lr * rbfConfig_.weightDecay * weights_(row, j) * dt;
    }
  }
}

// ── Update (main control cycle) ────────────────────────────────────────────
void AdaptiveEstimatorRbf::update(const ocs2::vector_t& state,
                                   const ocs2::vector_t& stateDes,
                                   ocs2::scalar_t dt) {
  if (dt <= 0.0 || dt > 0.1) {
    return;  // Skip invalid timesteps
  }

  // 1. Compute composite error sigma (6D)
  const vector6_t sigma = computeCompositeError(state, stateDes);

  // 2. Build input features chi
  const ocs2::vector_t chi = buildInputFeatures(state, stateDes);

  // 3. Forward pass: chi -> wrench estimate
  wrenchEstimate_ = forward(chi);

  // 4. Compute activations (reuse from forward pass for efficiency)
  const ocs2::vector_t activations = computeActivations(chi);

  // 5. Update weights via gradient descent
  updateWeights(sigma, activations, dt);

  // 6. Clamp wrench estimate to reasonable bounds
  for (int i = 0; i < 3; ++i) {
    wrenchEstimate_(i) = std::max(-rbfConfig_.maxForceEstimate,
                                   std::min(rbfConfig_.maxForceEstimate,
                                            wrenchEstimate_(i)));
  }
  for (int i = 3; i < 6; ++i) {
    wrenchEstimate_(i) = std::max(-rbfConfig_.maxTorqueEstimate,
                                   std::min(rbfConfig_.maxTorqueEstimate,
                                            wrenchEstimate_(i)));
  }

  // 7. Populate output
  lastOutput_.sigma = sigma;
  lastOutput_.adaptiveForce = wrenchEstimate_.head<3>();
  lastOutput_.adaptiveTorque = wrenchEstimate_.tail<3>();
}

// ── Get Output ─────────────────────────────────────────────────────────────
EstimatorOutput AdaptiveEstimatorRbf::getOutput() const {
  return lastOutput_;
}

}  // namespace adaptive
}  // namespace legged
