/******************************************************************************
 * AdaptiveEstimatorLegacy.cpp
 *
 * Implementation of the legacy ACLF-MPC estimator wrapper.
 * Delegates to the original AdaptiveParams / AdaptiveRegressor code.
 ******************************************************************************/

#include "legged_interface/adaptive/AdaptiveEstimatorLegacy.h"

#include <ocs2_core/math/MatrixOperations.h>
#include <ocs2_legged_robot/adaptive/AdaptiveParams.h>

#include <cmath>

namespace legged {
namespace adaptive {

// ── Helper: build 16x16 diagonal Gamma from scalar gains ──────────────────
static ocs2::matrix_t buildGammaMatrix(ocs2::scalar_t gammaMass,
                                        ocs2::scalar_t gammaCom,
                                        ocs2::scalar_t gammaInertia,
                                        ocs2::scalar_t gammaWrench) {
  constexpr int N = 16;
  ocs2::matrix_t Gamma = ocs2::matrix_t::Zero(N, N);

  // [0]   : mass
  Gamma(0, 0) = gammaMass;

  // [1-3] : first mass moment (CoM)
  for (int i = 1; i <= 3; ++i) {
    Gamma(i, i) = gammaCom;
  }

  // [4-9] : inertia (6 elements)
  for (int i = 4; i <= 9; ++i) {
    Gamma(i, i) = gammaInertia;
  }

  // [10-12] : constant force
  for (int i = 10; i <= 12; ++i) {
    Gamma(i, i) = gammaWrench;
  }

  // [13-15] : constant torque
  for (int i = 13; i <= 15; ++i) {
    Gamma(i, i) = gammaWrench;
  }

  return Gamma;
}

// ── Constructor ───────────────────────────────────────────────────────────
AdaptiveEstimatorLegacy::AdaptiveEstimatorLegacy(
    const LegacyEstimatorConfig& config)
    : AdaptiveEstimatorBase(config), legacyConfig_(config) {
  using namespace ocs2::legged_robot::adaptive;

  paramsPtr_ = std::make_unique<AdaptiveParams>();

  // Copy common config into AdaptiveParams
  paramsPtr_->Lambda_l = config.Lambda_l;
  paramsPtr_->Lambda_o = config.Lambda_o;
  paramsPtr_->KD = config.KD_diag.asDiagonal();

  // Build 16x16 Gamma from scalar gains
  paramsPtr_->Gamma = buildGammaMatrix(config.gammaMass, config.gammaCom,
                                       config.gammaInertia,
                                       config.gammaWrench);

  reset();
}

AdaptiveEstimatorLegacy::~AdaptiveEstimatorLegacy() = default;

// ── Reset ──────────────────────────────────────────────────────────────────
void AdaptiveEstimatorLegacy::reset() {
  paramsPtr_->pi_hat.setZero();
  lastOutput_ = EstimatorOutput{};
}

// ── Update ─────────────────────────────────────────────────────────────────
void AdaptiveEstimatorLegacy::update(const ocs2::vector_t& state,
                                     const ocs2::vector_t& stateDes,
                                     ocs2::scalar_t dt) {
  using namespace ocs2::legged_robot::adaptive;

  if (dt <= 0.0 || dt > 0.1) {
    return;  // Skip invalid timesteps
  }

  // ── 1. Compute composite error sigma (6D) ─────────────────────────────
  const ocs2::vector6_t sigma = computeCompositeError(state, stateDes);

  // ── 2. Extract current velocity and orientation from state ─────────────
  // State layout: [v_com(3), L/m(3)=angularMomentum/mass, p(3), euler(3)]
  const ocs2::vector3_t omega = state.segment<3>(3);  // L/m ~= I*omega/m ~= omega for small I/m
  // Actually L/m is NOT omega. But the old code also uses this approximation.
  // For accurate computation we'd need I⁻¹ * L. We keep the old code's
  // behavior for consistency.

  // ── 3. Compute regressor matrix Yu (6x16) ──────────────────────────────
  // The old code sets v_dot_pr = 0 and omega_dot_r = 0 (no reference accel).
  const ocs2::vector3_t v_dot_pr = ocs2::vector3_t::Zero();
  const ocs2::vector3_t omega_dot_r = ocs2::vector3_t::Zero();
  const ocs2::vector3_t omega_r = omega;  // reference ang vel = current (for simplicity)

  const ocs2::matrix_t Yu = AdaptiveRegressor::computeRegressor(
      v_dot_pr, omega, omega_dot_r, omega_r, config_.gravity);

  // ── 4. Update adaptive parameters ──────────────────────────────────────
  // pi_dot = Gamma * Yu^T * sigma
  // pi_hat += pi_dot * dt
  ocs2::legged_robot::adaptive::updateAdaptiveParams(
      paramsPtr_->pi_hat, paramsPtr_->Gamma, sigma, Yu, dt);

  // ── 5. Compute adaptive wrench from params ─────────────────────────────
  lastOutput_.sigma = sigma;
  lastOutput_.adaptiveForce = paramsPtr_->computeAdaptiveForce(
      v_dot_pr, config_.gravity);
  lastOutput_.adaptiveTorque = paramsPtr_->computeAdaptiveTorque(
      omega, omega_dot_r);
}

// ── Get Output ─────────────────────────────────────────────────────────────
EstimatorOutput AdaptiveEstimatorLegacy::getOutput() const {
  return lastOutput_;
}

}  // namespace adaptive
}  // namespace legged
