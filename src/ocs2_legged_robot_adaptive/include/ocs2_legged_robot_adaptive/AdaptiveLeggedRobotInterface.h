#pragma once

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveDisturbanceEstimator.h"
#include "ocs2_legged_robot_adaptive/constraint/AdaptiveClfConstraint.h"

namespace ocs2 {
namespace legged_robot_adaptive {

class AdaptiveLeggedRobotInterface final : public RobotInterface {
 public:
  AdaptiveLeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                               bool useHardFrictionConeConstraint = false);

  ~AdaptiveLeggedRobotInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const Initializer& getInitializer() const override { return baseInterfacePtr_->getInitializer(); }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return baseInterfacePtr_->getReferenceManagerPtr(); }

  const ddp::Settings& ddpSettings() const { return baseInterfacePtr_->ddpSettings(); }
  const mpc::Settings& mpcSettings() const { return baseInterfacePtr_->mpcSettings(); }
  const rollout::Settings& rolloutSettings() const { return baseInterfacePtr_->rolloutSettings(); }
  const sqp::Settings& sqpSettings() const { return baseInterfacePtr_->sqpSettings(); }
  const ipm::Settings& ipmSettings() const { return baseInterfacePtr_->ipmSettings(); }

  const vector_t& getInitialState() const { return baseInterfacePtr_->getInitialState(); }
  const RolloutBase& getRollout() const { return baseInterfacePtr_->getRollout(); }
  const legged_robot::ModelSettings& modelSettings() const { return baseInterfacePtr_->modelSettings(); }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return baseInterfacePtr_->getCentroidalModelInfo(); }
  std::shared_ptr<legged_robot::SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const {
    return baseInterfacePtr_->getSwitchedModelReferenceManagerPtr();
  }

  std::shared_ptr<AdaptiveDisturbanceEstimator> getAdaptiveDisturbanceEstimatorPtr() const { return estimatorPtr_; }

 private:
  struct AdaptiveSettings {
    AdaptiveDisturbanceEstimator::Config estimatorConfig;
    AdaptiveClfConstraint::Config clfConfig;
    RelaxedBarrierPenalty::Config clfSoftPenaltyConfig{0.1, 5.0};
    scalar_t inputBiasWeight = 2e-3;
    bool useSoftClfConstraint = true;
  };

  static AdaptiveSettings loadAdaptiveSettings(const std::string& taskFile);

  std::unique_ptr<legged_robot::LeggedRobotInterface> baseInterfacePtr_;
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<AdaptiveDisturbanceEstimator> estimatorPtr_;
};

}  // namespace legged_robot_adaptive
}  // namespace ocs2
