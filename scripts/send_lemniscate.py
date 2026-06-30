#!/usr/bin/env python3
"""
Send a figure-eight (lemniscate) velocity reference for Go2 Gazebo validation.

Matches the MATLAB simulation reference:
  x_r(t) = a * sin(omega_t * t)
  y_r(t) = b * sin(omega_t * t) * cos(omega_t * t)

with a=1.0m, b=0.65m, T_ref=16s.

Publishes to /cmd_vel as Twist messages.
"""
import rospy
import argparse
import numpy as np
from geometry_msgs.msg import Twist


def lemniscate_velocity(t, a=1.0, b=0.65, T_ref=16.0):
    """Compute figure-eight reference velocity at time t."""
    omega = 2.0 * np.pi / T_ref
    # Position
    # x = a * sin(omega*t)
    # y = b * sin(omega*t) * cos(omega*t)
    # Velocity (derivative)
    vx = a * omega * np.cos(omega * t)
    vy = b * omega * (np.cos(omega * t)**2 - np.sin(omega * t)**2)
    # Angular velocity: track tangent direction
    # heading = atan2(vy, vx), but for simplicity send zero
    wz = 0.0
    return vx, vy, wz


def main():
    parser = argparse.ArgumentParser(description="Send figure-eight reference to Go2")
    parser.add_argument("--duration", type=float, default=30.0, help="Run duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate (Hz)")
    parser.add_argument("--T_ref", type=float, default=16.0, help="Reference period (s)")
    parser.add_argument("--a", type=float, default=1.0, help="X amplitude (m)")
    parser.add_argument("--b", type=float, default=0.65, help="Y amplitude (m)")
    args = parser.parse_args()

    rospy.init_node("lemniscate_reference", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(args.rate)
    t_start = rospy.Time.now().to_sec()

    print(f"[Lemniscate] Starting figure-eight reference: "
          f"a={args.a}m, b={args.b}m, T_ref={args.T_ref}s, "
          f"duration={args.duration}s, rate={args.rate}Hz")

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_start
        if t > args.duration:
            # Stop command
            msg = Twist()
            pub.publish(msg)
            print(f"[Lemniscate] Finished after {t:.1f}s")
            break

        vx, vy, wz = lemniscate_velocity(t, args.a, args.b, args.T_ref)
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    main()
