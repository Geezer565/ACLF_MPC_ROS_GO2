#include "ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

#include "ocs2_legged_robot_adaptive/cost/AdaptiveInputCost.h"
#include "ocs2_legged_robot_adaptive/package_path.h"

namespace ocs2 {
namespace legged_robot_adaptive {
namespace {

template <typename T>
void loadIfPresent(const boost::property_tree::ptree& pt, const std::string& key, T& value) {
  if (auto optionalValue = pt.get_optional<T>(key)) {
    value = *optionalValue;
  }
}

}  // namespace

AdaptiveLeggedRobotInterface::AdaptiveLeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool useHardFrictionConeConstraint)
    : baseInterfacePtr_(new legged_robot::LeggedRobotInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint)),
      problemPtr_(new OptimalControlProblem(baseInterfacePtr_->getOptimalControlProblem())) {
  const auto adaptiveSettings = loadAdaptiveSettings(taskFile);
  estimatorPtr_ = std::make_shared<AdaptiveDisturbanceEstimator>(adaptiveSettings.estimatorConfig);

  const auto referenceManagerPtr = baseInterfacePtr_->getSwitchedModelReferenceManagerPtr();
  const auto info = baseInterfacePtr_->getCentroidalModelInfo();

  // Additional CLF-style inequality term driven by online disturbance estimates.
  auto adaptiveClfConstraint =
      std::make_unique<AdaptiveClfConstraint>(*estimatorPtr_, *referenceManagerPtr, info, adaptiveSettings.clfConfig);
  if (adaptiveSettings.useSoftClfConstraint) {
    problemPtr_->softConstraintPtr->add(
        "adaptiveClf",
        std::make_unique<StateInputSoftConstraint>(std::move(adaptiveClfConstraint),
                                                   std::make_unique<RelaxedBarrierPenalty>(adaptiveSettings.clfSoftPenaltyConfig)));
  } else {
    problemPtr_->inequalityConstraintPtr->add("adaptiveClf", std::move(adaptiveClfConstraint));
  }

  // Bias nominal contact forces using the current adaptive estimate.
  matrix_t adaptiveR = matrix_t::Zero(info.inputDim, info.inputDim);
  const size_t contactForceDim = 3 * info.numThreeDofContacts;
  adaptiveR.topLeftCorner(contactForceDim, contactForceDim).diagonal().array() = adaptiveSettings.inputBiasWeight;
  problemPtr_->costPtr->add("adaptiveInputBias",
                            std::make_unique<AdaptiveInputBiasCost>(adaptiveR, info, *referenceManagerPtr, *estimatorPtr_));
}

AdaptiveLeggedRobotInterface::AdaptiveSettings AdaptiveLeggedRobotInterface::loadAdaptiveSettings(const std::string& taskFile) {
  AdaptiveSettings settings;

  const auto mergeSettings = [&](const boost::property_tree::ptree& pt) {
    loadIfPresent(pt, "adaptive_estimator.lambdaPosition", settings.estimatorConfig.lambdaPosition);
    loadIfPresent(pt, "adaptive_estimator.lambdaOrientation", settings.estimatorConfig.lambdaOrientation);
    loadIfPresent(pt, "adaptive_estimator.forceAdaptationGain", settings.estimatorConfig.forceAdaptationGain);
    loadIfPresent(pt, "adaptive_estimator.torqueAdaptationGain", settings.estimatorConfig.torqueAdaptationGain);
    loadIfPresent(pt, "adaptive_estimator.maxForceEstimate", settings.estimatorConfig.maxForceEstimate);
    loadIfPresent(pt, "adaptive_estimator.maxTorqueEstimate", settings.estimatorConfig.maxTorqueEstimate);

    loadIfPresent(pt, "adaptive_clf.lambdaPosition", settings.clfConfig.lambdaPosition);
    loadIfPresent(pt, "adaptive_clf.lambdaOrientation", settings.clfConfig.lambdaOrientation);
    loadIfPresent(pt, "adaptive_clf.clfRate", settings.clfConfig.clfRate);
    loadIfPresent(pt, "adaptive_clf.epsilon", settings.clfConfig.epsilon);
    loadIfPresent(pt, "adaptive_clf.softConstraintMu", settings.clfSoftPenaltyConfig.mu);
    loadIfPresent(pt, "adaptive_clf.softConstraintDelta", settings.clfSoftPenaltyConfig.delta);
    loadIfPresent(pt, "adaptive_clf.inputBiasWeight", settings.inputBiasWeight);
    loadIfPresent(pt, "adaptive_clf.useSoftConstraint", settings.useSoftClfConstraint);
  };

  // 1) Load package-local defaults if present.
  try {
    boost::property_tree::ptree defaultPt;
    boost::property_tree::read_info(getPath() + "/config/adaptive_settings.info", defaultPt);
    mergeSettings(defaultPt);
  } catch (...) {
    // Keep hard-coded defaults.
  }

  // 2) Override by task file values when provided.
  boost::property_tree::ptree taskPt;
  boost::property_tree::read_info(taskFile, taskPt);
  mergeSettings(taskPt);

  return settings;
}

}  // namespace legged_robot_adaptive
}  // namespace ocs2
