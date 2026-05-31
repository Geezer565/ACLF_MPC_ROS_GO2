#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveDisturbanceEstimator.h"

namespace ocs2 {
namespace legged_robot_adaptive {

class AdaptiveClfConstraint final : public StateInputConstraint {
 public:
  // CLF-like inequality term inspired by the paper:
  //   h = epsilon - sigma^T * d_hat - c * ||sigma||^2 >= 0
  // where sigma stacks translational and rotational sliding variables.
  struct Config {
    scalar_t lambdaPosition = 2.0;
    scalar_t lambdaOrientation = 1.5;
    scalar_t clfRate = 0.6;
    scalar_t epsilon = 0.05;
  };

  AdaptiveClfConstraint(const AdaptiveDisturbanceEstimator& estimator, const legged_robot::SwitchedModelReferenceManager& referenceManager,
                        CentroidalModelInfo info, Config config);

  ~AdaptiveClfConstraint() override = default;
  AdaptiveClfConstraint* clone() const override { return new AdaptiveClfConstraint(*this); }

  bool isActive(scalar_t time) const override { return true; }
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  AdaptiveClfConstraint(const AdaptiveClfConstraint& other) = default;

  const AdaptiveDisturbanceEstimator* estimatorPtr_;
  const legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  const CentroidalModelInfo info_;
  const Config config_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
