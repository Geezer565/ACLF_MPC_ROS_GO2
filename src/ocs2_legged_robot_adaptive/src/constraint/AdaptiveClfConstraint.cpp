#include "ocs2_legged_robot_adaptive/constraint/AdaptiveClfConstraint.h"

#include <utility>

namespace ocs2 {
namespace legged_robot_adaptive {

AdaptiveClfConstraint::AdaptiveClfConstraint(const AdaptiveDisturbanceEstimator& estimator,
                                             const legged_robot::SwitchedModelReferenceManager& referenceManager, CentroidalModelInfo info,
                                             Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      estimatorPtr_(&estimator),
      referenceManagerPtr_(&referenceManager),
      info_(std::move(info)),
      config_(std::move(config)) {}

vector_t AdaptiveClfConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const vector_t desiredState = referenceManagerPtr_->getTargetTrajectories().getDesiredState(time);
  const auto estimate = estimatorPtr_->getActiveEstimate();

  const legged_robot::vector3_t positionError = state.segment<3>(6) - desiredState.segment<3>(6);
  const legged_robot::vector3_t comVelocityError = state.segment<3>(0) - desiredState.segment<3>(0);
  const legged_robot::vector3_t orientationError = state.segment<3>(9) - desiredState.segment<3>(9);
  const legged_robot::vector3_t angularMomentumError = state.segment<3>(3) - desiredState.segment<3>(3);

  const legged_robot::vector3_t sigmaPosition = comVelocityError + config_.lambdaPosition * positionError;
  const legged_robot::vector3_t sigmaOrientation = angularMomentumError + config_.lambdaOrientation * orientationError;

  const scalar_t sigmaNormSq = sigmaPosition.squaredNorm() + sigmaOrientation.squaredNorm();
  const scalar_t disturbanceProjection =
      sigmaPosition.dot(estimate.externalForceInWorld) + sigmaOrientation.dot(estimate.externalTorqueInWorld);

  vector_t value(1);
  value(0) = config_.epsilon - disturbanceProjection - config_.clfRate * sigmaNormSq;
  return value;
}

VectorFunctionLinearApproximation AdaptiveClfConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const vector_t& input,
                                                                                const PreComputation& preComp) const {
  const vector_t desiredState = referenceManagerPtr_->getTargetTrajectories().getDesiredState(time);
  const auto estimate = estimatorPtr_->getActiveEstimate();

  const legged_robot::vector3_t positionError = state.segment<3>(6) - desiredState.segment<3>(6);
  const legged_robot::vector3_t comVelocityError = state.segment<3>(0) - desiredState.segment<3>(0);
  const legged_robot::vector3_t orientationError = state.segment<3>(9) - desiredState.segment<3>(9);
  const legged_robot::vector3_t angularMomentumError = state.segment<3>(3) - desiredState.segment<3>(3);

  const legged_robot::vector3_t sigmaPosition = comVelocityError + config_.lambdaPosition * positionError;
  const legged_robot::vector3_t sigmaOrientation = angularMomentumError + config_.lambdaOrientation * orientationError;

  VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, input, preComp);
  approximation.dfdx = matrix_t::Zero(1, state.size());
  approximation.dfdu = matrix_t::Zero(1, input.size());

  const legged_robot::vector3_t gradSigmaPosition =
      estimate.externalForceInWorld + 2.0 * config_.clfRate * sigmaPosition;
  const legged_robot::vector3_t gradSigmaOrientation =
      estimate.externalTorqueInWorld + 2.0 * config_.clfRate * sigmaOrientation;

  approximation.dfdx.block<1, 3>(0, 0) = -gradSigmaPosition.transpose();
  approximation.dfdx.block<1, 3>(0, 6) = -(config_.lambdaPosition * gradSigmaPosition).transpose();
  approximation.dfdx.block<1, 3>(0, 3) = -gradSigmaOrientation.transpose();
  approximation.dfdx.block<1, 3>(0, 9) = -(config_.lambdaOrientation * gradSigmaOrientation).transpose();

  return approximation;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
