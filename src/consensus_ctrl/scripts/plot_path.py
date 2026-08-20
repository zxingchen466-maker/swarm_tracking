#!/usr/bin/env python3
"""实时画 XY 轨迹形状"""
import rospy
import matplotlib.pyplot as plt
from consensus_ctrl.msg import TrajRef

xs, ys = [], []

def cb(msg):
    xs.append(msg.pos.x)
    ys.append(msg.pos.y)

rospy.init_node('plot_path')
rospy.Subscriber('/robot_1/ref', TrajRef, cb)

# 订阅 30 秒收集数据
rospy.sleep(30)

plt.figure(figsize=(6, 6))
plt.plot(xs, ys, 'b-', linewidth=2)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.title('Reference Trajectory (fig8)')
plt.grid(True)
plt.show()
