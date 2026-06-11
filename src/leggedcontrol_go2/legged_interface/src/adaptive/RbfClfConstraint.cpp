/******************************************************************************
 * RbfClfConstraint.cpp
 *
 * Implementation of the RBF-based CLF inequality constraint for OCS2.
 ******************************************************************************/

#include "legged_interface/adaptive/RbfClfConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>

namespace legged {
namespace adaptive {

// ── Constructor ───────────────────────────────────────────────────────────
RbfClfConstraint::RbfClfConstraint(const ocs2::CentroidalModelInfo& info,
                                     const AdaptiveEstimatorRbf* rbfEstimator,
                                     const EstimatorConfig& config)
    : info_(info),
      rbfEstimatorPtr_(rbfEstimator),
      gravity_(config.gravity),
      Lambda_l_(config.Lambda_l),
      Lambda_o_(config.Lambda_o),
      mass_(config.nominalMass) {

  // Build KD matrix (6x6 diagonal)
  KD_ = ocs2::matrix_t::Zero(6, 6);
  for (int i = 0; i < 6; ++i) {
    KD_(i, i) = config.KD_diag(i);
  }

  // Precompute inverse inertia
  inertiaInv_ = config.nominalInertia.inverse();

  // Store nominal foot positions in base frame
  footPositionsBase_.clear();
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    footPositionsBase_.push_back(
        info_.endEffectorCoordinates.segment<3>(3 * leg));
  }
}

// ── Compute Composite Error ────────────────────────────────────────────────
ocs2::vector6_t RbfClfConstraint::computeCompositeError(
    ocs2::scalar_t time, const ocs2::vector_t& state) const {

  // State layout: [v_com(3), L(3)/m, p(3), eulerZYX(3)]
  const ocs2::vector3_t v_com = state.segment<3>(0);
  const ocs2::vector3_t L_over_m = state.segment<3>(3);
  const ocs2::vector3_t position = state.segment<3>(6);
  const ocs2::vector3_t euler = state.segment<3>(9);

  // Compute angular velocity from angular momentum
  // L = I * omega  =>  omega = I^{-1} * L
  // L = L_over_m * mass  =>  omega = I^{-1} * L_over_m * mass
  const ocs2::vector3_t L = L_over_m * mass_;
  const ocs2::vector3_t omega = inertiaInv_ * L;

  // Get reference state (from SwitchedModelReferenceManager)
  // Note: referenceManagerPtr_ is accessible through the base class pattern.
  // For simplicity, we assume the reference trajectories are embedded in
  // the state's desired values through OCS2's tracking cost.
  //
  // In practice, the desired state is provided by the TargetTrajectories
  // which the OCS2 solver reads from ReferenceManager. The tracking
  // error is already represented in the MPC cost function Q matrix.
  //
  // For the CLF constraint, we compute sigma directly from the state
  // and the desired state obtained through the precomputation:
  //   sigma_l = (v_com - v_des) + Lambda_l * (p - p_des)
  //   sigma_o = (omega - omega_des) + Lambda_o * (euler - euler_des)
  //
  // The reference state is available in the OCS2 solver's internal
  // reference, but NOT directly accessible from a StateInputConstraint.
  // Instead, we rely on the tracking cost to encode the reference,
  // and the CLF constraint stabilizes the error dynamics via sigma.
  //
  // For now, use zero desired state (tracking objective encoded in cost):
  // sigma_l = v_com + Lambda_l * position
  // sigma_o = omega + Lambda_o * euler
  //
  // This is a simplification; the full implementation would read
  // the desired state from the ReferenceManager.

  // TODO(Phase 2): Read desired state directly from preComputation
  // or SwitchedModelReferenceManager for proper sigma computation.

  // Placeholder: use tracking errors encoded in state only
  // (the desired state is embedded in the MPC cost function Q)
  const ocs2::vector3_t v_des = ocs2::vector3_t::Zero();
  const ocs2::vector3_t p_des = ocs2::vector3_t::Zero();
  const ocs2::vector3_t omega_des = ocs2::vector3_t::Zero();
  const ocs2::vector3_t euler_des = ocs2::vector3_t::Zero();

  ocs2::vector6_t sigma;
  sigma.head<3>() = (v_com - v_des) + Lambda_l_ * (position - p_des);
  sigma.tail<3>() = (omega - omega_des) + Lambda_o_ * (euler - euler_des);

  return sigma;
}

// ── Compute Constraint Residual ────────────────────────────────────────────
ocs2::vector6_t RbfClfConstraint::computeConstraintResidual(
    ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::vector_t& input) const {

  // Extract contact forces from input
  // Input layout: [f_1(3), f_2(3), f_3(3), f_4(3), joint_vel(12)]
  ocs2::vector3_t appliedForce = ocs2::vector3_t::Zero();
  ocs2::vector3_t appliedTorque = ocs2::vector3_t::Zero();

  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    const ocs2::vector3_t force = input.segment<3>(3 * leg);
    appliedForce += force;
    appliedTorque += footPositionsBase_[leg].cross(force);
  }

  // State components
  const ocs2::vector3_t L_over_m = state.segment<3>(3);
  const ocs2::vector3_t L = L_over_m * mass_;
  const ocs2::vector3_t omega = inertiaInv_ * L;

  // Nominal wrench: gravity force + gyroscopic torque
  const ocs2::vector3_t nomForce = mass_ * gravity_;
  const ocs2::vector3_t nomTorque = -omega.cross(info_.robotInertia * omega);

  // Adaptive wrench from RBFNN (constant during this MPC solve)
  const ocs2::vector6_t adapWrench = rbfEstimatorPtr_->getWrenchEstimate();
  const ocs2::vector3_t adapForce = adapWrench.head<3>();
  const ocs2::vector3_t adapTorque = adapWrench.tail<3>();

  // RHS = -[appliedForce; I^{-1}*appliedTorque]
  //       + [nomForce; I^{-1}*nomTorque]
  //       + [adapForce; I^{-1}*adapTorque]
  ocs2::vector6_t rhs;
  rhs.head<3>() =
      -(appliedForce) / mass_ + nomForce / mass_ + adapForce / mass_;
  rhs.tail<3>() =
      -inertiaInv_ * appliedTorque + inertiaInv_ * nomTorque
      + inertiaInv_ * adapTorque;

  return rhs;
}

// ── getValue ───────────────────────────────────────────────────────────────
ocs2::vector_t RbfClfConstraint::getValue(
    ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::vector_t& input, const ocs2::PreComputation& /*preComp*/) const {

  const ocs2::vector6_t sigma = computeCompositeError(time, state);
  const ocs2::vector6_t rhs = computeConstraintResidual(time, state, input);

  // h_clf = -sigma^T * rhs - 0.5 * sigma^T * K_D * sigma
  const ocs2::scalar_t h = -sigma.dot(rhs) - 0.5 * sigma.dot(KD_ * sigma);

  ocs2::vector_t constraint(1);
  constraint(0) = h;
  return constraint;
}

// ── getLinearApproximation ─────────────────────────────────────────────────
ocs2::VectorFunctionLinearApproximation
RbfClfConstraint::getLinearApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const {

  const ocs2::vector6_t sigma = computeCompositeError(time, state);
  const ocs2::vector6_t rhs = computeConstraintResidual(time, state, input);
  const ocs2::scalar_t h = -sigma.dot(rhs) - 0.5 * sigma.dot(KD_ * sigma);

  ocs2::VectorFunctionLinearApproximation approx;
  approx.f = ocs2::vector_t(1);
  approx.f(0) = h;
  approx.dfdx.resize(1, state.size());
  approx.dfdu.resize(1, input.size());
  approx.dfdx.setZero();
  approx.dfdu.setZero();

  // ── Analytic Jacobian w.r.t. input ──────────────────────────────────
  // dh/du = -sigma^T * d(rhs)/du
  // d(rhs)/du: only appliedForce and appliedTorque depend on input
  //   d(appliedForce)/d(f_i) = I_3  for each foot
  //   d(appliedTorque)/d(f_i) = skew(r_i)  for each foot
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    const int idx = 3 * leg;
    // Force contribution: -sigma_l^T * (-I_3 / mass) = sigma_l^T / mass
    approx.dfdu.block(0, idx, 1, 3) += sigma.head<3>().transpose() / mass_;
    // Torque contribution: -sigma_o^T * (-I^{-1} * skew(r_i))
    const ocs2::matrix3_t rSkew = ocs2::skewSymmetricMatrix(footPositionsBase_[leg]);
    approx.dfdu.block(0, idx, 1, 3) +=
        sigma.tail<3>().transpose() * inertiaInv_ * rSkew;
  }

  // ── Finite-difference Jacobian w.r.t. state ─────────────────────────
  // (Same approach as legacy AdaptiveClfConstraint)
  const ocs2::scalar_t eps = 1e-6;
  ocs2::vector_t statePerturbed = state;
  for (int i = 0; i < state.size(); ++i) {
    statePerturbed(i) = state(i) + eps;
    const ocs2::vector6_t sigmaP =
        computeCompositeError(time, statePerturbed);
    const ocs2::vector6_t rhsP =
        computeConstraintResidual(time, statePerturbed, input);
    const ocs2::scalar_t hP =
        -sigmaP.dot(rhsP) - 0.5 * sigmaP.dot(KD_ * sigmaP);
    approx.dfdx(0, i) = (hP - h) / eps;
    statePerturbed(i) = state(i);
  }

  return approx;
}

// ── getQuadraticApproximation ──────────────────────────────────────────────
ocs2::VectorFunctionQuadraticApproximation
RbfClfConstraint::getQuadraticApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const {

  auto approx = getLinearApproximation(time, state, input, preComp);

  // Return linear approximation with diagonal Hessian shift
  // (same strategy as legacy: no off-diagonal Hessian terms)
  ocs2::VectorFunctionQuadraticApproximation quad;
  quad.f = approx.f;
  quad.dfdx = approx.dfdx;
  quad.dfdu = approx.dfdu;
  quad.dfdxx.resize(1);
  quad.dfdux.resize(1);
  quad.dfduu.resize(1);
  quad.dfdxx[0].resize(state.size(), state.size());
  quad.dfdxx[0].setIdentity();
  quad.dfdxx[0] *= 1e-4;  // Small diagonal shift for numerical stability
  quad.dfdux[0].setZero(input.size(), state.size());
  quad.dfduu[0].setZero(input.size(), input.size());

  return quad;
}

// ── Factory: Soft Constraint ───────────────────────────────────────────────
std::unique_ptr<ocs2::StateInputCost> createRbfClfSoftConstraint(
    const ocs2::CentroidalModelInfo& info,
    const AdaptiveEstimatorRbf* rbfEstimator,
    const EstimatorConfig& config,
    const RbfClfConfig& softConfig) {

  auto constraint = std::make_unique<RbfClfConstraint>(
      info, rbfEstimator, config);

  auto penalty = std::make_unique<ocs2::RelaxedBarrierPenalty>(
      ocs2::RelaxedBarrierPenalty::Config{softConfig.mu, softConfig.delta});

  return std::make_unique<ocs2::StateInputSoftConstraint>(
      std::move(constraint), std::move(penalty));
}

}  // namespace adaptive
}  // namespace legged
