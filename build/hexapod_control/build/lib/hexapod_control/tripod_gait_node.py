#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TripodGaitNode(Node):
    def __init__(self):
        super().__init__('tripod_gait_node')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/hexapod_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'hip_1_joint', 'knee_1_joint', 'ankle_1_joint',
            'hip_2_joint', 'knee_2_joint', 'ankle_2_joint',
            'hip_3_joint', 'knee_3_joint', 'ankle_3_joint',
            'hip_4_joint', 'knee_4_joint', 'ankle_4_joint',
            'hip_5_joint', 'knee_5_joint', 'ankle_5_joint',
            'hip_6_joint', 'knee_6_joint', 'ankle_6_joint'
        ]

        self.timer = self.create_timer(2.0, self.publish_gait)
        self.phase = 0

        self.get_logger().info("Tripod gait node with continuous sweep started.")

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        forward_mag = -0.3   # forward hip rotation
        backward_mag = 0.3 # backward hip rotation
        knee_down = -0.3
        knee_up = 0.2

        if self.phase == 0:
            swing_legs = [1, 3, 5]  # Tripod A
            stance_legs = [2, 4, 6] # Tripod B
        else:
            swing_legs = [2, 4, 6]  # Tripod B
            stance_legs = [1, 3, 5] # Tripod A

        # Two points per phase: start & end
        for t in [0.0, 1.0]:
            positions = [0.0] * 18

            # Swing legs: backward → forward
            for leg in swing_legs:
                hip_idx = (leg - 1) * 3
                knee_idx = hip_idx + 1
                ankle_idx = hip_idx + 2

                start_hip = backward_mag
                end_hip = forward_mag

                hip_pos = start_hip + (end_hip - start_hip) * t
                positions[hip_idx] = hip_pos
                positions[knee_idx] = knee_up
                positions[ankle_idx] = 0.0

            # Stance legs: forward → backward
            for leg in stance_legs:
                hip_idx = (leg - 1) * 3
                knee_idx = hip_idx + 1
                ankle_idx = hip_idx + 2

                start_hip = forward_mag
                end_hip = backward_mag

                hip_pos = start_hip + (end_hip - start_hip) * t
                positions[hip_idx] = hip_pos
                positions[knee_idx] = knee_down
                positions[ankle_idx] = 0.0

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(t)
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.phase = 1 - self.phase


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
