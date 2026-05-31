//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged {
using namespace ocs2;
using namespace centroidal_model;
class SafetyChecker {
 public:
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    return checkOrientation(observation);
  }

 protected:
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    // 仿真放宽到 160 度 (M_PI=180)，避免机器人晃动时误杀控制器
    if (pose(5) > 0.9 * M_PI || pose(5) < -0.9 * M_PI) {
      std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace legged
