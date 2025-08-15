#!/usr/bin/env python3
import math
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

        # === Tunable gait params (can be changed via ROS params) ===
        self.declare_parameter('phase_time', 1.0)         # seconds per phase (two phases per cycle)
        self.declare_parameter('steps', 6)                # waypoints per phase
        self.declare_parameter('hip_forward', -0.3)       # forward hip angle (rad)
        self.declare_parameter('hip_backward', 0.3)       # backward hip angle (rad)
        self.declare_parameter('knee_up', 0.1)            # swing knee (rad)
        self.declare_parameter('knee_down', -0.2)         # stance knee (rad)
        self.declare_parameter('touchdown_start', 0.5)    # fraction of swing when knee starts lowering
        self.declare_parameter('yaw_bias', 0.1)           # + biases left hips forward, right backward (or vice-versa depending on indexing)

        # Per-leg hip gains and offsets (index legs 1..6)
        for leg in range(1, 7):
            self.declare_parameter(f'hip_gain_{leg}', 1.0)     # amplitude scale
            self.declare_parameter(f'hip_offset_{leg}', 0.0)   # constant bias (rad)

        phase_time = float(self.get_parameter('phase_time').value)
        self.timer = self.create_timer(phase_time * 2.0, self.publish_gait)  # keep callback cadence = full cycle
        self.phase = 0

        self.get_logger().info("Tripod gait node (smooth + drift correction) started.")

    # Helpers to read params quickly
    def p(self, name): return self.get_parameter(name).value

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Read parameters (so you can tweak at runtime)
        forward_mag   = float(self.p('hip_forward'))
        backward_mag  = float(self.p('hip_backward'))
        knee_up       = float(self.p('knee_up'))
        knee_down     = float(self.p('knee_down'))
        touchdown_t0  = float(self.p('touchdown_start'))  # e.g., 0.8
        steps         = int(self.p('steps'))
        phase_time    = float(self.p('phase_time'))
        yaw_bias      = float(self.p('yaw_bias'))

        # Tripod grouping
        if self.phase == 0:
            swing_legs = [1, 3, 5]  # Tripod A
            stance_legs = [2, 4, 6] # Tripod B
        else:
            swing_legs = [2, 4, 6]  # Tripod B
            stance_legs = [1, 3, 5] # Tripod A

        # Build waypoints across this phase
        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0  # 0 → 1
            positions = [0.0] * 18

            # Smooth (cosine) easing
            eased_t = 0.5 - 0.5 * math.cos(math.pi * t)

            # ---- Swing legs: backward → forward, knee up then gently down ----
            for leg in swing_legs:
                hip_idx = (leg - 1) * 3
                knee_idx = hip_idx + 1
                ankle_idx = hip_idx + 2

                # Base symmetrical sweep
                base = backward_mag + (forward_mag - backward_mag) * eased_t

                # Apply per-leg amplitude gain and constant offset
                gain   = float(self.p(f'hip_gain_{leg}'))
                offset = float(self.p(f'hip_offset_{leg}'))

                # Apply yaw_bias: bias left vs right sides (here legs 1,3,5 assumed left; 2,4,6 right.
                # Swap if your model is opposite.)
                side_bias = yaw_bias if leg in [1, 3, 5] else -yaw_bias

                hip_pos = (base * gain) + offset + side_bias
                positions[hip_idx] = hip_pos

                # Knee touchdown smoothing
                if t < touchdown_t0:
                    knee_pos = knee_up
                else:
                    denom = max(1e-6, (1.0 - touchdown_t0))
                    drop_ratio = (t - touchdown_t0) / denom  # 0→1
                    knee_pos = knee_up + (knee_down - knee_up) * drop_ratio
                positions[knee_idx] = knee_pos
                positions[ankle_idx] = 0.0

            # ---- Stance legs: forward → backward, knee stays down ----
            for leg in stance_legs:
                hip_idx = (leg - 1) * 3
                knee_idx = hip_idx + 1
                ankle_idx = hip_idx + 2

                base = forward_mag + (backward_mag - forward_mag) * eased_t
                gain   = float(self.p(f'hip_gain_{leg}'))
                offset = float(self.p(f'hip_offset_{leg}'))
                side_bias = yaw_bias if leg in [1, 3, 5] else -yaw_bias

                hip_pos = (base * gain) + offset + side_bias
                positions[hip_idx] = hip_pos
                positions[knee_idx] = knee_down
                positions[ankle_idx] = 0.0

            # Timing for this waypoint within the phase
            point = JointTrajectoryPoint()
            point.positions = positions
            tsec = t * phase_time
            point.time_from_start.sec = int(tsec)
            point.time_from_start.nanosec = int((tsec % 1.0) * 1e9)
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
