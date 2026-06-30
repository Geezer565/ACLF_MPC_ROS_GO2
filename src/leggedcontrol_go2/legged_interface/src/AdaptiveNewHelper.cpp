// Direct implementation of the new ACLF (6D disturbance) logic.
// Does NOT include ocs2_legged_robot_adaptive headers to avoid
// namespace conflicts with legged_interface's own types.
//
#include <ocs2_core/cost/QuadraticStateInputCost.h>
// Implements:
//   1. AdaptiveDisturbanceEstimator: d̂ -= Γ·σ·dt
//   2. AdaptiveClfConstraint: h = ε - σᵀd̂ - c·‖σ‖² ≥ 0
//   3. AdaptiveInputBiasCost: distribute f_ext to stance feet

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/common/utils.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

namespace legged {
namespace new_adaptive {

using namespace ocs2;
using namespace legged_robot;
using namespace centroidal_model;

// ---- Lightweight disturbance estimator (replaces AdaptiveDisturbanceEstimator) ----
struct DisturbanceEstimator {
  struct Config {
    scalar_t lambdaPosition = 2.0;
    scalar_t lambdaOrientation = 1.5;
    scalar_t forceAdaptationGain = 3.0;
    scalar_t torqueAdaptationGain = 1.5;
    scalar_t maxForceEstimate = 150.0;
    scalar_t maxTorqueEstimate = 80.0;
  };

  vector3_t externalForceInWorld = vector3_t::Zero();
  vector3_t externalTorqueInWorld = vector3_t::Zero();
  scalar_t lastUpdateTime = std::numeric_limits<scalar_t>::quiet_NaN();
  Config config;

  void update(scalar_t currentTime, const vector_t& state, const vector_t& desiredState) {
    if (std::isfinite(lastUpdateTime)) {
      scalar_t dt = std::max<scalar_t>(0.0, currentTime - lastUpdateTime);
      vector3_t posErr = state.segment<3>(6) - desiredState.segment<3>(6);
      vector3_t velErr = state.segment<3>(0) - desiredState.segment<3>(0);
      vector3_t oriErr = state.segment<3>(9) - desiredState.segment<3>(9);
      vector3_t angErr = state.segment<3>(3) - desiredState.segment<3>(3);

      vector3_t sigmaP = velErr + config.lambdaPosition * posErr;
      vector3_t sigmaO = angErr + config.lambdaOrientation * oriErr;

      externalForceInWorld -= config.forceAdaptationGain * dt * sigmaP;
      externalTorqueInWorld -= config.torqueAdaptationGain * dt * sigmaO;

      // Clamp
      auto clamp = [](vector3_t& v, scalar_t maxNorm) {
        scalar_t n = v.norm();
        if (n > maxNorm && n > 1e-8) v *= maxNorm / n;
      };
      clamp(externalForceInWorld, config.maxForceEstimate);
      clamp(externalTorqueInWorld, config.maxTorqueEstimate);
    }
    lastUpdateTime = currentTime;
  }
};

// ---- CLF constraint: h = ε - σᵀd̂ - c·‖σ‖² ≥ 0 ----
class AdaptiveClf : public StateInputConstraint {
 public:
  struct Config {
    scalar_t lambdaPosition = 2.0;
    scalar_t lambdaOrientation = 1.5;
    scalar_t clfRate = 0.6;
    scalar_t epsilon = 0.05;
  };

  AdaptiveClf(const DisturbanceEstimator& estimator,
              const SwitchedModelReferenceManager& refManager,
              CentroidalModelInfo info, Config cfg)
      : StateInputConstraint(ConstraintOrder::Linear),
        estimator_(&estimator), refManager_(&refManager),
        info_(std::move(info)), config_(std::move(cfg)) {}

  bool isActive(scalar_t) const override { return true; }
  size_t getNumConstraints(scalar_t) const override { return 1; }
  AdaptiveClf* clone() const override { return new AdaptiveClf(*this); }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t&,
                    const PreComputation&) const override {
    vector_t desiredState = refManager_->getTargetTrajectories().getDesiredState(time);
    vector3_t pErr = state.segment<3>(6) - desiredState.segment<3>(6);
    vector3_t vErr = state.segment<3>(0) - desiredState.segment<3>(0);
    vector3_t oErr = state.segment<3>(9) - desiredState.segment<3>(9);
    vector3_t wErr = state.segment<3>(3) - desiredState.segment<3>(3);

    vector3_t sp = vErr + config_.lambdaPosition * pErr;
    vector3_t so = wErr + config_.lambdaOrientation * oErr;

    scalar_t sigmaNormSq = sp.squaredNorm() + so.squaredNorm();
    scalar_t disturbanceProj = sp.dot(estimator_->externalForceInWorld)
                             + so.dot(estimator_->externalTorqueInWorld);

    vector_t value(1);
    value(0) = config_.epsilon - disturbanceProj - config_.clfRate * sigmaNormSq;
    return value;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation approx;
    approx.f = getValue(time, state, input, preComp);
    approx.dfdx = matrix_t::Zero(1, state.size());
    approx.dfdu = matrix_t::Zero(1, input.size());
    return approx;  // dfdu = 0: does not increase QP complexity
  }

 private:
  AdaptiveClf(const AdaptiveClf&) = default;
  const DisturbanceEstimator* estimator_;
  const SwitchedModelReferenceManager* refManager_;
  const CentroidalModelInfo info_;
  const Config config_;
};

// ---- Input bias cost: distribute f_ext across stance feet ----
class AdaptiveInputBias : public QuadraticStateInputCost {
 public:
  AdaptiveInputBias(matrix_t R, CentroidalModelInfo info,
                    const SwitchedModelReferenceManager& refManager,
                    const DisturbanceEstimator& estimator)
      : QuadraticStateInputCost(matrix_t::Zero(info.stateDim, info.stateDim), std::move(R)),
        info_(std::move(info)), refManager_(&refManager), estimator_(&estimator) {}

  AdaptiveInputBias* clone() const override { return new AdaptiveInputBias(*this); }

 private:
  AdaptiveInputBias(const AdaptiveInputBias&) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state,
                                                       const vector_t& input,
                                                       const TargetTrajectories&) const override {
    auto contactFlags = refManager_->getContactFlags(time);
    vector_t nominalInput = legged_robot::weightCompensatingInput(info_, contactFlags);
    size_t numStanceLegs = legged_robot::numberOfClosedContacts(contactFlags);

    if (numStanceLegs > 0) {
      vector3_t compensationPerFoot = -estimator_->externalForceInWorld / static_cast<scalar_t>(numStanceLegs);
      for (size_t i = 0; i < contactFlags.size(); i++) {
        if (contactFlags[i]) {
          centroidal_model::getContactForces(nominalInput, i, info_) += compensationPerFoot;
        }
      }
    }
    return {vector_t::Zero(state.size()), input - nominalInput};
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* refManager_;
  const DisturbanceEstimator* estimator_;
};

// ---- Public setup function ----
void setupNewAdaptive(
    OptimalControlProblem& problem,
    std::shared_ptr<DisturbanceEstimator>& estimator,
    const SwitchedModelReferenceManager& refManager,
    const CentroidalModelInfo& info,
    const DisturbanceEstimator::Config& estimatorCfg,
    const AdaptiveClf::Config& clfCfg,
    const RelaxedBarrierPenalty::Config& barrierCfg,
    scalar_t inputBiasWeight, bool useSoft)
{
    estimator = std::make_shared<DisturbanceEstimator>();
    estimator->config = estimatorCfg;

    if (useSoft) {
        problem.softConstraintPtr->add("adaptiveClfNew",
            std::make_unique<StateInputSoftConstraint>(
                std::make_unique<AdaptiveClf>(*estimator, refManager, info, clfCfg),
                std::make_unique<RelaxedBarrierPenalty>(barrierCfg)));
    } else {
        problem.inequalityConstraintPtr->add("adaptiveClfNew",
            std::make_unique<AdaptiveClf>(*estimator, refManager, info, clfCfg));
    }

    matrix_t adaptiveR = matrix_t::Zero(info.inputDim, info.inputDim);
    const size_t contactForceDim = 3 * info.numThreeDofContacts;
    adaptiveR.topLeftCorner(contactForceDim, contactForceDim).diagonal().array() = inputBiasWeight;
    problem.costPtr->add("adaptiveInputBias",
        std::make_unique<AdaptiveInputBias>(adaptiveR, info, refManager, *estimator));
}

}  // namespace new_adaptive
}  // namespace legged
