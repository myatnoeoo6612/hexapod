#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TripodGaitIKNode(Node):
    def __init__(self):
        super().__init__('tripod_gait_ik_node')

        # Publisher to controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/hexapod_joint_trajectory_controller/joint_trajectory',
            10
        )

        # === Joint names ===
        self.joint_names = [
            'hip_1_joint', 'knee_1_joint',
            'hip_2_joint', 'knee_2_joint',
            'hip_3_joint', 'knee_3_joint',
            'hip_4_joint', 'knee_4_joint',
            'hip_5_joint', 'knee_5_joint',
            'hip_6_joint', 'knee_6_joint'
        ]

        # === Leg geometry (meters) ===
        self.L1 = 0.024   # hip → knee
        self.L2 = 0.140   # knee → foot
        self.z_ground = -0.150  # hip center → ground

        # === Gait parameters ===
        self.declare_parameter('phase_time', 1.0)
        self.declare_parameter('steps', 6)
        self.declare_parameter('step_length', 0.05)   # forward distance
        self.declare_parameter('step_height', 0.03)   # lift height

        # Forward direction per leg (sign convention)
        self.forward_dir = {
            1: -1,  # RF
            2: -1,  # RM
            3: -1,  # RR
            4: -1,  # LF
            5: +1,  # LM
            6: +1   # LR
        }

        # Timer for publishing
        phase_time = float(self.get_parameter('phase_time').value)
        self.timer = self.create_timer(phase_time, self.publish_gait)
        self.phase = 0

        self.get_logger().info("Tripod gait IK node started.")

    def ik_solve(self, x, z):
        """Solve hip & knee angles for target (x, z)"""
        D = (x**2 + z**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if D < -1.0 or D > 1.0:
            return None, None  # out of reach

        theta_knee = math.atan2(math.sqrt(1 - D**2), D)  # elbow-down
        theta_hip = math.atan2(z, x) - math.atan2(self.L2*math.sin(theta_knee),
                                                  self.L1 + self.L2*math.cos(theta_knee))
        return theta_hip, theta_knee

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        steps = int(self.get_parameter('steps').value)
        phase_time = float(self.get_parameter('phase_time').value)
        step_length = float(self.get_parameter('step_length').value)
        step_height = float(self.get_parameter('step_height').value)

        # Tripod groups
        if self.phase == 0:
            swing_legs = [2, 4, 6]  # group B
            stance_legs = [1, 3, 5] # group A
        else:
            swing_legs = [1, 3, 5]  # group A
            stance_legs = [2, 4, 6] # group B

        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0
            eased_t = 0.5 - 0.5 * math.cos(math.pi * t)

            positions = [0.0] * len(self.joint_names)

            # Swing legs: follow arc
            for leg in swing_legs:
                hip_idx = (leg - 1) * 2
                knee_idx = hip_idx + 1
                sign = self.forward_dir[leg]

                x = sign * step_length * (2*eased_t - 1)
                z = self.z_ground + step_height * math.sin(math.pi * t)

                theta_hip, theta_knee = self.ik_solve(x, z)
                if theta_hip is None:
                    self.get_logger().warn(f"IK out of reach for leg {leg} at x={x:.3f}, z={z:.3f}")
                    theta_hip, theta_knee = 0.0, 0.5

                positions[hip_idx] = theta_hip
                positions[knee_idx] = theta_knee

            # Stance legs: move backward near ground
            for leg in stance_legs:
                hip_idx = (leg - 1) * 2
                knee_idx = hip_idx + 1
                sign = self.forward_dir[leg]

                x = -sign * step_length * (2*eased_t - 1)
                z = self.z_ground

                theta_hip, theta_knee = self.ik_solve(x, z)
                if theta_hip is None:
                    self.get_logger().warn(f"IK out of reach for leg {leg} at x={x:.3f}, z={z:.3f}")
                    theta_hip, theta_knee = 0.0, 0.5

                positions[hip_idx] = theta_hip
                positions[knee_idx] = theta_knee

            # Add waypoint
            point = JointTrajectoryPoint()
            point.positions = positions
            tsec = (i + 1) * (phase_time / steps)
            point.time_from_start.sec = int(tsec)
            point.time_from_start.nanosec = int((tsec % 1.0) * 1e9)
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.phase = 1 - self.phase


def main(args=None):
    rclpy.init(args=args)
    node = TripodGaitIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
