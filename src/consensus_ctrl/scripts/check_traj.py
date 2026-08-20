#!/usr/bin/env python3
"""检查解析速度是否等于位置数值差分"""
import rospy
from consensus_ctrl.msg import TrajRef
import numpy as np

class Checker:
    def __init__(self):
        self.last = None
        rospy.Subscriber('/robot_1/ref', TrajRef, self.cb)

    def cb(self, msg):
        if self.last is None:
            self.last = msg
            return
        dt = (msg.header.stamp - self.last.header.stamp).to_sec()
        if dt <= 0:
            return
        num_vx = (msg.pos.x - self.last.pos.x) / dt
        num_vy = (msg.pos.y - self.last.pos.y) / dt
        ana_vx = msg.vel.x
        ana_vy = msg.vel.y
        err_x = abs(num_vx - ana_vx) / max(abs(ana_vx), 1e-6) * 100
        err_y = abs(num_vy - ana_vy) / max(abs(ana_vy), 1e-6) * 100
        print("vx: 数值=%.4f 解析=%.4f 误差=%.2f%% | vy: 数值=%.4f 解析=%.4f 误差=%.2f%%"
              % (num_vx, ana_vx, err_x, num_vy, ana_vy, err_y))
        self.last = msg

if __name__ == '__main__':
    rospy.init_node('check_traj')
    Checker()
    rospy.spin()
