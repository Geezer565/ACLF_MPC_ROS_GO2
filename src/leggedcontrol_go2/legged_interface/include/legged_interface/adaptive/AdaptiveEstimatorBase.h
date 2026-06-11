/******************************************************************************
 * AdaptiveEstimatorBase.h
 *
 * Abstract interface for adaptive disturbance estimation in ACLF-MPC.
 * Supports strategy pattern: Legacy (Paper A, 16-param Slotine-Li) and
 * RBF (Paper B, Dual-RBFNN) estimators share this interface, enabling
 * runtime switching for comparison experiments.
 *
 * References:
 *   Paper A — Minniti et al., "Adaptive CLF-MPC With Application To
 *             Quadrupedal Robots", IEEE RA-L, 2021.
 *   Paper B — Liu et al., "Adaptive MPC-Based Multi-Terrain Trajectory
 *             Tracking Framework for Mobile Spherical Robots", IEEE/ASME
 *             TMECH, 2025.
 ******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace legged {
namespace adaptive {

/**
 * Configuration common to all adaptive estimators.
 */
struct EstimatorConfig {
  // Sliding surface gains: sigma = v_tilde + Lambda * p_tilde
  ocs2::matrix3_t Lambda_l{ocs2::matrix3_t::Identity() * 5.0};
  ocs2::matrix3_t Lambda_o{ocs2::matrix3_t::Identity() * 5.0};

  // CLF damping matrix (6x6 diagonal)
  ocs2::vector6_t KD_diag{ocs2::vector6_t::Zero()};

  // Gravity vector (world frame)
  ocs2::vector3_t gravity{0.0, 0.0, -9.81};

  // Nominal robot mass (kg)
  ocs2::scalar_t nominalMass{6.921};

  // Nominal robot inertia about CoM (base frame)
  ocs2::matrix3_t nominalInertia{ocs2::matrix3_t::Identity()};
};

/**
 * Result of one adaptive estimation cycle.
 */
struct EstimatorOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Adaptive force in world frame (3D)
  ocs2::vector3_t adaptiveForce{ocs2::vector3_t::Zero()};

  /// Adaptive torque in base frame (3D)
  ocs2::vector3_t adaptiveTorque{ocs2::vector3_t::Zero()};

  /// 6D composite error sigma = [sigma_l; sigma_o]
  ocs2::vector6_t sigma{ocs2::vector6_t::Zero()};

  /// Current CLF constraint value h_clf (for monitoring)
  ocs2::scalar_t clfValue{0.0};
};

/**
 * Abstract base class for adaptive disturbance estimators.
 *
 * Each implementation provides:
 *   1. update()      — called at control rate (100Hz) to update estimates
 *   2. getOutput()   — returns current wrench estimate and diagnostic info
 *   3. reset()       — reinitialize internal state
 *
 * The adaptive wrench enters the control pipeline via:
 *   LeggedRobotPreComputation::setAdaptiveWrench(f_u, t_u)
 * and is consumed by the WBC layer as feed-forward compensation.
 */
class AdaptiveEstimatorBase {
 public:
  using Ptr = std::shared_ptr<AdaptiveEstimatorBase>;
  using ConstPtr = std::shared_ptr<const AdaptiveEstimatorBase>;

  explicit AdaptiveEstimatorBase(const EstimatorConfig& config)
      : config_(config) {}
  virtual ~AdaptiveEstimatorBase() = default;

  /**
   * Update the estimator with latest measurements.
   *
   * @param state        Current measured state [v_com(3), L/m(3), p(3), euler(3)]
   * @param stateDes     Desired state (same layout as state)
   * @param dt           Time step since last update (seconds)
   */
  virtual void update(const ocs2::vector_t& state,
                      const ocs2::vector_t& stateDes, ocs2::scalar_t dt) = 0;

  /**
   * Get the current estimator output.
   */
  virtual EstimatorOutput getOutput() const = 0;

  /**
   * Reset internal state (parameters, weights) to initial values.
   */
  virtual void reset() = 0;

  /**
   * Human-readable name for logging / mode display.
   */
  virtual std::string getName() const = 0;

  /**
   * Compute composite error sigma from current state and reference.
   *
   * sigma_l = v_tilde + Lambda_l * p_tilde
   * sigma_o = omega_tilde + Lambda_o * euler_tilde
   *
   * @param state    Current measured state
   * @param stateDes Desired state
   * @return         6D composite error [sigma_l; sigma_o]
   */
  ocs2::vector6_t computeCompositeError(const ocs2::vector_t& state,
                                         const ocs2::vector_t& stateDes) const;

 protected:
  EstimatorConfig config_;
};

}  // namespace adaptive
}  // namespace legged
