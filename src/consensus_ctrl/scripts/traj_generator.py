#!/usr/bin/env python3
"""traj_generator.py - 发布参考轨迹 (circle / fig8)"""

import rospy
import numpy as np
from consensus_ctrl.msg import TrajRef
from geometry_msgs.msg import Vector3


class TrajGenerator:
    def __init__(self):
        # ---- 从 yaml 读参数 ----
        self.rate    = rospy.get_param('~rate', 50)
        self.traj    = rospy.get_param('~traj_type', 'circle')
        self.t_ramp  = rospy.get_param('~t_ramp', 3.0)
        self.yaw_mod = rospy.get_param('~yaw_mode', 'fixed')

        if self.traj == 'fig8':
            self.A = rospy.get_param('~fig8/A', 1.5)
            self.T = rospy.get_param('~fig8/T', 25.0)
        else:
            self.R = rospy.get_param('~circle/R', 1.5)
            self.T = rospy.get_param('~circle/T', 20.0)

        # ---- 发布器 ----
        self.pub = rospy.Publisher('ref', TrajRef, queue_size=10)
        self.t_start = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.cb)

    def cb(self, event):
        t = (rospy.Time.now() - self.t_start).to_sec()
        omega = 2.0 * np.pi / self.T

        # 平滑因子（cosine ramp）
        s = 0.5 * (1.0 - np.cos(np.pi * t / self.t_ramp)) if t < self.t_ramp else 1.0

        # ---- 解析轨迹 + 解析导数 ----
        if self.traj == 'fig8':
            px = self.A * np.sin(omega * t)
            py = self.A * np.sin(omega * t) * np.cos(omega * t)
            vx = self.A * omega * np.cos(omega * t)
            vy = self.A * omega * np.cos(2.0 * omega * t)
            ax = -self.A * omega**2 * np.sin(omega * t)
            ay = -2.0 * self.A * omega**2 * np.sin(2.0 * omega * t)
        else:   # circle
            px =  self.R * np.cos(omega * t)
            py =  self.R * np.sin(omega * t)
            vx = -self.R * omega * np.sin(omega * t)
            vy =  self.R * omega * np.cos(omega * t)
            ax = -self.R * omega**2 * np.cos(omega * t)
            ay = -self.R * omega**2 * np.sin(omega * t)

        # 平滑加在速度与加速度上（位置保持解析值）
        vx *= s;  vy *= s
        ax *= s;  ay *= s

        # yaw
        yaw = np.arctan2(vy, vx) if self.yaw_mod == 'tangent' else 0.0

        # ---- 发布 ----
        msg = TrajRef()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.pos = Vector3(px, py, yaw)
        msg.vel = Vector3(vx, vy, 0.0)
        msg.acc = Vector3(ax, ay, 0.0)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('traj_generator')
    TrajGenerator()
    rospy.spin()
