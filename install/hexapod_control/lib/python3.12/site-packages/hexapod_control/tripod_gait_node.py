#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TripodGaitNode(Node):
    def __init__(self):
        super().__init__('tripod_gait_node')

        # Publisher to your controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/hexapod_joint_trajectory_controller/joint_trajectory',
            10
        )

        # === Joint names (must match controllers.yaml) ===
        self.joint_names = [
            'hip_1_joint', 'knee_1_joint',
            'hip_2_joint', 'knee_2_joint',
            'hip_3_joint', 'knee_3_joint',
            'hip_4_joint', 'knee_4_joint',
            'hip_5_joint', 'knee_5_joint',
            'hip_6_joint', 'knee_6_joint'
        ]

        # === Tunable gait parameters ===
        self.declare_parameter('phase_time', 1.0)   # seconds per phase
        self.declare_parameter('steps', 6)          # waypoints per phase
        self.declare_parameter('hip_swing', 0.3)    # rad swing range
        self.declare_parameter('knee_lift', 0.2)    # rad lift range

        # === Joint limits from your config ===
        self.hip_min = -0.523
        self.hip_max =  0.523
        self.hip_init = 0.0

        self.knee_down = 0.5
        self.knee_up = 0.7
        self.knee_init = 0.5

        # === Forward swing direction per leg (based on your config) ===
        # +1 = forward swing is positive hip rotation
        # -1 = forward swing is negative hip rotation
        # Forward swing direction per leg (based on your latest config)
        self.forward_dir = {
            1: +1,  # RF
            2: -1,  # RM
            3: +1,  # RR
            4: -1,  # LF
            5: -1,  # LM
            6: -1   # LR
        }


        phase_time = float(self.get_parameter('phase_time').value)
        self.timer = self.create_timer(phase_time, self.publish_gait)
        self.phase = 0

        self.get_logger().info("Tripod gait node started with per-leg forward direction mapping.")

    def clamp(self, x, lo, hi):
        """Clamp joint command to limits"""
        return max(lo, min(x, hi))

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        steps = int(self.get_parameter('steps').value)
        phase_time = float(self.get_parameter('phase_time').value)
        hip_swing = float(self.get_parameter('hip_swing').value)
        knee_lift = float(self.get_parameter('knee_lift').value)

        # Tripod groups
        if self.phase == 0:
            swing_legs = [1, 3, 5]  # A
            stance_legs = [2, 4, 6] # B
        else:
            swing_legs = [2, 4, 6]  # B
            stance_legs = [1, 3, 5] # A

        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0  # normalized [0..1]
            eased_t = 0.5 - 0.5 * math.cos(math.pi * t)  # cosine ease

            positions = [0.0] * len(self.joint_names)

            # Swing legs: hip moves forward, knee lifts
            for leg in swing_legs:
                hip_idx = (leg - 1) * 2
                knee_idx = hip_idx + 1
                sign = self.forward_dir[leg]

                hip_pos = self.hip_init + sign * hip_swing * (2 * eased_t - 1)
                knee_pos = self.knee_down + (self.knee_up - self.knee_down) * math.sin(math.pi * t)

                positions[hip_idx] = self.clamp(hip_pos, self.hip_min, self.hip_max)
                positions[knee_idx] = self.clamp(knee_pos, self.knee_down, self.knee_up)

            # Stance legs: hip moves backward, knee stays low
            for leg in stance_legs:
                hip_idx = (leg - 1) * 2
                knee_idx = hip_idx + 1
                sign = self.forward_dir[leg]

                hip_pos = self.hip_init - sign * hip_swing * (2 * eased_t - 1)
                knee_pos = self.knee_down  # grounded

                positions[hip_idx] = self.clamp(hip_pos, self.hip_min, self.hip_max)
                positions[knee_idx] = self.knee_down

            # Add waypoint
            point = JointTrajectoryPoint()
            point.positions = positions
            tsec = t * phase_time
            point.time_from_start.sec = int(tsec)
            point.time_from_start.nanosec = int((tsec % 1.0) * 1e9)
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.phase = 1 - self.phase  # switch tripod


def main(args=None):
    rclpy.init(args=args)
    node = TripodGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
