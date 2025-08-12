#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class TripodGaitNode(Node):
    def __init__(self):
        super().__init__('tripod_gait_node')

        # legs and joints
        self.legs = [1,2,3,4,5,6]
        self.joint_types = ['hip','knee','ankle']

        # publishers for each joint controller command topic:
        # controllers are named: <joint>_<leg>_joint_position_controller
        self.pubs = {}
        for leg in self.legs:
            for jt in self.joint_types:
                ctrl = f'{jt}_{leg}_joint_position_controller'
                topic = f'/{ctrl}/command'   # controller publishes command on this topic
                # create publisher
                self.pubs[(jt,leg)] = self.create_publisher(Float64, topic, 10)

        # gait params (tweak)
        self.freq = 0.8        # gait cycles per second
        self.hip_swing = 0.5   # radians (hip forward/back amplitude)
        self.knee_lift = 0.6   # radians knee lift during swing
        self.started = False

        # small startup delay to let controllers spawn
        self.create_timer(2.0, self._start_gait)

    def _start_gait(self):
        if not self.started:
            # cancel the startup timer by setting flag and create main timer
            self.started = True
            self.get_logger().info('Starting tripod gait')
            self.t0 = self.get_clock().now().nanoseconds / 1e9
            self.create_timer(0.02, self._gait_loop)

    def _gait_loop(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        phase = (t * self.freq) % 1.0  # 0..1
        # tripod A (odd legs) swing in phase [0,0.5), B (even) swing in [0.5,1)
        if phase < 0.5:
            s = phase * 2.0
            swing_legs = [1,3,5]
            stance_legs = [2,4,6]
        else:
            s = (phase - 0.5) * 2.0
            swing_legs = [2,4,6]
            stance_legs = [1,3,5]

        # swing interpolation (smooth)
        for leg in swing_legs:
            # hip: move from -hip_swing to +hip_swing across swing (simple sinus)
            hip_cmd = -self.hip_swing * math.cos(math.pi * s) + 0.0
            # knee: lift with sin shape
            knee_cmd = self.knee_lift * math.sin(math.pi * s)
            self.pubs[('hip',leg)].publish(Float64(data=hip_cmd))
            self.pubs[('knee',leg)].publish(Float64(data=knee_cmd))

        # stance legs hold (on ground)
        for leg in stance_legs:
            hip_cmd = 0.0
            knee_cmd = 0.0
            self.pubs[('hip',leg)].publish(Float64(data=hip_cmd))
            self.pubs[('knee',leg)].publish(Float64(data=knee_cmd))

        # ankles: keep 0 for now
        for leg in self.legs:
            self.pubs[('ankle',leg)].publish(Float64(data=0.0))


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
