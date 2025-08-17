#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import matplotlib.pyplot as plt
from collections import deque
import networkx as nx   # <-- for network visualization

plt.ion()  # interactive plotting

class HopfCPGNode(Node):
    def __init__(self):
        super().__init__('hopf_cpg_tripod')

        # ---- CPG parameters ----
        self.alpha = 5.0
        self.mu = 1.0
        self.freq_hz = 1.5
        self.duty = 0.6
        self.k_coupling = 8.0
        self.dt = 0.01

        self.hip_amp = math.radians(25)   # hip swing amplitude
        self.knee_amp = math.radians(20)  # knee lift amplitude
        self.knee_phase_lead = math.pi / 2

        # Tripod groups
        self.A = [0, 4, 2]   # Tripod A
        self.B = [3, 1, 5]   # Tripod B
        self.N = 6

        # Drift correction
        self.hip_gain = np.array([1.0] * self.N)

        # State variables
        rng = np.random.default_rng(42)
        self.x = rng.normal(size=self.N)
        self.y = rng.normal(size=self.N)

        # ROS interfaces
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/hexapod_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'hip_1_joint','knee_1_joint','ankle_1_joint',
            'hip_2_joint','knee_2_joint','ankle_2_joint',
            'hip_3_joint','knee_3_joint','ankle_3_joint',
            'hip_4_joint','knee_4_joint','ankle_4_joint',
            'hip_5_joint','knee_5_joint','ankle_5_joint',
            'hip_6_joint','knee_6_joint','ankle_6_joint'
        ]

        self.timer = self.create_timer(self.dt, self.update)

        # --- Buffers ---
        self.win_seconds = 5.0
        self.maxlen = int(self.win_seconds / self.dt)
        self.time_data = deque(maxlen=self.maxlen)
        self.hip_data = [deque(maxlen=self.maxlen) for _ in range(self.N)]
        self.x_data = [deque(maxlen=self.maxlen) for _ in range(self.N)]
        self.y_data = [deque(maxlen=self.maxlen) for _ in range(self.N)]
        self.gait_data = [deque(maxlen=self.maxlen) for _ in range(self.N)]

        # --- Figure with 4 panels ---
        self.fig = plt.figure(figsize=(12, 8))
        self.ax1 = self.fig.add_subplot(2, 2, 1)  # hips
        self.ax2 = self.fig.add_subplot(2, 2, 2)  # x,y activations
        self.ax3 = self.fig.add_subplot(2, 2, 3)  # gait raster
        self.ax4 = self.fig.add_subplot(2, 2, 4)  # network diagram

        # Panel 1: hips
        self.hip_lines = [self.ax1.plot([], [], label=f"Leg {i+1}")[0] for i in range(self.N)]
        self.ax1.set_ylabel('Hip angle (rad) + offset')
        self.ax1.legend(loc='upper right', ncol=3, fontsize='small')
        self.ax1.set_ylim(-1.5, self.N*0.3 + 1.5)

        # Panel 2: neuron x,y
        self.x_lines = [self.ax2.plot([], [], lw=1.2, label=f"x{i+1}")[0] for i in range(self.N)]
        self.y_lines = [self.ax2.plot([], [], ls='--', lw=0.8, label=f"y{i+1}")[0] for i in range(self.N)]
        self.ax2.set_ylabel('CPG states (x,y)')
        self.ax2.legend(loc='upper right', ncol=3, fontsize='small')
        self.ax2.set_ylim(-2.0, 2.0)

        # Panel 3: gait raster
        init_mat = np.zeros((self.N, self.maxlen))
        self.gait_im = self.ax3.imshow(
            init_mat,
            aspect='auto',
            origin='lower',
            interpolation='nearest',
            cmap='gray_r',
            vmin=0, vmax=1   # <-- fix here
        )
        self.ax3.set_ylabel('Leg')
        self.ax3.set_xlabel('Time (sliding window)')
        self.ax3.set_yticks(range(self.N))
        self.ax3.set_yticklabels([f"{i+1}" for i in range(self.N)])


        # Panel 4: network diagram
        self.G = nx.DiGraph()
        for i in range(self.N):
            self.G.add_node(i)
        for i in range(self.N):
            for j in range(self.N):
                if i != j:
                    self.G.add_edge(i, j)
        self.pos = nx.circular_layout(self.G)

        plt.tight_layout()
        self.t = 0.0

        self.get_logger().info("Hopf CPG tripod gait node with 4-panel live plot started.")

    # ---------- CPG math ----------
    def phase_bias(self, i, j):
        if (i in self.A and j in self.A) or (i in self.B and j in self.B):
            return 0.0
        else:
            return math.pi

    def leg_omega(self, phi):
        eps = 1e-6
        rho = (1.0 - self.duty + eps) / (self.duty + eps)
        omega_stance = (2*math.pi*self.freq_hz) / (self.duty + (1.0-self.duty)*rho)
        omega_swing = rho * omega_stance
        return np.where(np.sin(phi) >= 0.0, omega_swing, omega_stance)

    def rotate_pair(self, xj, yj, delta):
        cd = math.cos(delta)
        sd = math.sin(delta)
        return cd * xj - sd * yj, sd * xj + cd * yj

    # ---------- Main update ----------
    def update(self):
        # --- Hopf update ---
        phi = np.arctan2(self.y, self.x)
        omega_vec = self.leg_omega(phi)
        r2 = self.x**2 + self.y**2

        dx = self.alpha * (self.mu - r2) * self.x - omega_vec * self.y
        dy = self.alpha * (self.mu - r2) * self.y + omega_vec * self.x

        for i in range(self.N):
            cx = cy = 0.0
            for j in range(self.N):
                if i == j: continue
                delta = self.phase_bias(i, j)
                xr, yr = self.rotate_pair(self.x[j], self.y[j], delta)
                cx += (xr - self.x[i])
                cy += (yr - self.y[i])
            dx[i] += self.k_coupling * cx
            dy[i] += self.k_coupling * cy

        self.x += dx * self.dt
        self.y += dy * self.dt

        # --- Map to joints ---
        hips = self.hip_amp * self.x * self.hip_gain
        xk = self.x * math.cos(self.knee_phase_lead) - self.y * math.sin(self.knee_phase_lead)
        knees = self.knee_amp * np.maximum(0.0, -xk)
        ankles = np.zeros(self.N)

        # Publish trajectory
        positions = []
        for leg in range(self.N):
            positions += [hips[leg], knees[leg], ankles[leg]]
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        traj.points.append(point)
        try:
            self.publisher_.publish(traj)
        except Exception as e:
            self.get_logger().debug(f"Publish failed: {e}")

        # --- Gait state ---
        phi = np.arctan2(self.y, self.x)
        swing_bool = np.sin(phi) >= 0.0

        # --- Update buffers ---
        self.t += self.dt
        self.time_data.append(self.t)
        for i in range(self.N):
            self.hip_data[i].append(hips[i] + i*0.3)
            self.x_data[i].append(self.x[i])
            self.y_data[i].append(self.y[i])
            self.gait_data[i].append(1.0 if swing_bool[i] else 0.0)

        tarr = np.array(self.time_data)
        if tarr.size > 0:
            xmin, xmax = max(0.0, tarr[-1]-self.win_seconds), tarr[-1]+0.001

            # Hip plot
            self.ax1.set_xlim(xmin, xmax)
            for i in range(self.N):
                self.hip_lines[i].set_xdata(tarr)
                self.hip_lines[i].set_ydata(np.array(self.hip_data[i]))

            # x,y plot
            self.ax2.set_xlim(xmin, xmax)
            for i in range(self.N):
                self.x_lines[i].set_xdata(tarr)
                self.x_lines[i].set_ydata(np.array(self.x_data[i]))
                self.y_lines[i].set_xdata(tarr)
                self.y_lines[i].set_ydata(np.array(self.y_data[i]))

        # Gait raster
        mat = np.zeros((self.N, self.maxlen))
        for i in range(self.N):
            row = list(self.gait_data[i])
            if row:
                mat[i, -len(row):] = row
        
        self.gait_im.set_data(mat)
        self.gait_im.set_extent([xmin, xmax, 0, self.N])  # <-- new line


        # Network diagram
        self.ax4.clear()
        colors = ["green" if self.x[i] > 0 else "lightgray" for i in range(self.N)]
        nx.draw(self.G, pos=self.pos, ax=self.ax4,
                with_labels=True, node_color=colors,
                node_size=800, font_size=10, arrows=False)
        self.ax4.set_title("CPG Network")

        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HopfCPGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
