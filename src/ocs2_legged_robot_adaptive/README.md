# ocs2_legged_robot_adaptive

This package adds an adaptive CLF-MPC layer on top of the existing `ocs2_legged_robot` example.

## Added components

- `AdaptiveDisturbanceEstimator`:
  - Online update of external force/torque estimate using a sliding-variable adaptation law.
- `AdaptiveClfConstraint`:
  - CLF-like inequality term converted into an MPC constraint.
- `AdaptiveInputBiasCost`:
  - Biases nominal contact-force distribution using the adaptive estimate.
- `AdaptiveLeggedRobotInterface`:
  - Wraps `ocs2_legged_robot::LeggedRobotInterface`, reuses the original OCP, and injects adaptive terms.

## How to configure

The loader reads adaptive parameters from the same `task.info` used by the MPC node:

- `adaptive_estimator.*`
- `adaptive_clf.*`

You can copy the template blocks from:

- `config/adaptive_settings.info`

and append them to your active `task.info`.

If these fields are not present, built-in defaults are used.
