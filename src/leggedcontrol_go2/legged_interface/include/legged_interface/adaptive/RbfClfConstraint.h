/******************************************************************************
 * RbfClfConstraint.h
 *
 * OCS2 StateInputConstraint for the RBF-based CLF inequality.
 *
 * Constraint form (from Paper A Eq.11, adapted for RBFNN):
 *   h_clf = -sigma^T * RHS - 0.5 * sigma^T * K_D * sigma  >=  0
 *
 * where:
 *   RHS = -[appliedForce; I^{-1}*appliedTorque]
 *         + [nomForce; I^{-1}*nomTorque]
 *         + [rbfForce; I^{-1}*rbfTorque]
 *
 * The adaptive wrench [rbfForce; rbfTorque] is provided by the
 * AdaptiveEstimatorRbf and treated as constant during a single MPC solve.
 *
 * This is analogous to the legacy AdaptiveClfConstraint but uses
 * the RBFNN wrench estimate instead of Y_u * pi_hat_u.
 ******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>

#include "legged_interface/adaptive/AdaptiveEstimatorRbf.h"

namespace legged {
namespace adaptive {

/**
 * Configuration for the RBF CLF soft constraint.
 */
struct RbfClfConfig {
  ocs2::scalar_t clfWeight{10.0};   // Weight in MPC cost
  ocs2::scalar_t mu{0.1};           // Relaxed barrier parameter
  ocs2::scalar_t delta{5.0};        // Relaxed barrier delta
};

/**
 * CLF inequality constraint that uses RBFNN-estimated wrench.
 *
 * Registered as a relaxed-barrier soft constraint in the OCS2
 * optimal control problem, analogous to the legacy ACLF constraint.
 *
 * The constraint has analytic gradient w.r.t. input (contact forces)
 * and is finite-differenced w.r.t. state.
 */
class RbfClfConstraint final : public ocs2::StateInputConstraint {
 public:
  /**
   * Constructor.
   *
   * @param info           Centroidal model info (mass, inertia, foot positions)
   * @param rbfEstimator   RBF estimator providing the wrench estimate
   * @param config         Base estimator config (Lambda, KD, gravity)
   */
  RbfClfConstraint(const ocs2::CentroidalModelInfo& info,
                   const AdaptiveEstimatorRbf* rbfEstimator,
                   const EstimatorConfig& config);

  ~RbfClfConstraint() override = default;

  RbfClfConstraint* clone() const override { return new RbfClfConstraint(*this); }

  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                          const ocs2::vector_t& input,
                          const ocs2::PreComputation& preComp) const override;

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::vector_t& input,
      const ocs2::PreComputation& preComp) const override;

  ocs2::VectorFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::vector_t& input,
      const ocs2::PreComputation& preComp) const override;

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }

 private:
  /**
   * Compute composite error sigma = [v_tilde + Lambda_l*p_tilde;
   *                                    omega_tilde + Lambda_o*euler_tilde]
   */
  ocs2::vector6_t computeCompositeError(ocs2::scalar_t time,
                                         const ocs2::vector_t& state) const;

  /**
   * Compute the RHS vector (6D) of the CLF inequality.
   */
  ocs2::vector6_t computeConstraintResidual(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::vector_t& input) const;

  const ocs2::CentroidalModelInfo info_;
  const AdaptiveEstimatorRbf* rbfEstimatorPtr_;  // Non-owning pointer
  const ocs2::vector3_t gravity_;

  // Derived quantities
  ocs2::matrix3_t Lambda_l_;
  ocs2::matrix3_t Lambda_o_;
  ocs2::matrix_t KD_;
  ocs2::scalar_t mass_;
  ocs2::matrix3_t inertiaInv_;

  // Foot contact positions in base frame (nominal)
  std::vector<ocs2::vector3_t> footPositionsBase_;
};

/**
 * Factory: create a soft CLF constraint for the RBF mode.
 *
 * Wraps RbfClfConstraint in StateInputSoftConstraint with
 * RelaxedBarrierPenalty.
 */
std::unique_ptr<ocs2::StateInputCost> createRbfClfSoftConstraint(
    const ocs2::CentroidalModelInfo& info,
    const AdaptiveEstimatorRbf* rbfEstimator,
    const EstimatorConfig& config,
    const RbfClfConfig& softConfig);

}  // namespace adaptive
}  // namespace legged
