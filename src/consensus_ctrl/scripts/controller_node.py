#!/usr/bin/env python3
"""controller_node.py - ROS 外壳：订阅/定时器/限幅/坐标系转换/安全逻辑"""

import rospy
import math
import importlib
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from consensus_ctrl.msg import TrajRef, CtrlDebug


class ControllerNode:
    def __init__(self):
        # ---- 参数 ----
        self.rate = rospy.get_param('~rate', 50)
        self.ctrl_type = rospy.get_param('~controller_type', 'pd')
        self.vmax = rospy.get_param('~vmax', 1.0)
        self.wmax = rospy.get_param('~wmax', 2.0)
        self.timeout = rospy.get_param('~timeout', 0.5)

        # ---- 动态加载控制器类（yaml 里 controller_type 决定）----
        ctrl_params = rospy.get_param('~ctrl_params', {})
        mod = importlib.import_module('controllers.' + self.ctrl_type)
        cls = getattr(mod, self.ctrl_type.upper() + 'Controller')
        self.controller = cls(ctrl_params)

        # ---- 状态 ----
        self.odom_msg = None
        self.ref_msg = None
        self.last_odom_t = None
        self.last_ref_t = None
        self.last_cb_t = None
        self.state = None

        # ---- 发布/订阅 ----
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('ctrl_debug', CtrlDebug, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('ref', TrajRef, self.ref_cb)

        # ---- 定时器 ----
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.control_loop)

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.last_odom_t = rospy.Time.now()
        q = msg.pose.pose.orientation
        (_, _, yaw) = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'wz': msg.twist.twist.angular.z,
        }

    def ref_cb(self, msg):
        self.ref_msg = msg
        self.last_ref_t = rospy.Time.now()

    def control_loop(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_cb_t).to_sec() if self.last_cb_t else 0.02
        self.last_cb_t = now

        # ---- 安全逻辑：odom/ref 超时 →零速 ----
        if self.last_odom_t is None or (now - self.last_odom_t).to_sec() > self.timeout:
            self.publish_zero()
            rospy.logwarn_throttle(1.0, "odom 超时，发布零速")
            return
        if self.last_ref_t is None or (now - self.last_ref_t).to_sec() > self.timeout:
            self.publish_zero()
            rospy.logwarn_throttle(1.0, "ref 超时，发布零速")
            return

        # ---- 组装 ref 字典 ----
        r = self.ref_msg
        ref = {
            'pos': (r.pos.x, r.pos.y, r.pos.z),
            'vel': (r.vel.x, r.vel.y, r.vel.z),
            'acc': (r.acc.x, r.acc.y, r.acc.z),
        }

        # ---- 调控制器（世界系）----
        cmd_w, debug = self.controller.compute(self.state, ref, dt)

        # ---- 世界系 →车体系 ----
        yaw = self.state['yaw']
        vx_body = math.cos(yaw) * cmd_w[0] + math.sin(yaw) * cmd_w[1]
        vy_body = -math.sin(yaw) * cmd_w[0] + math.cos(yaw) * cmd_w[1]
        wz_body = cmd_w[2]

        # ---- 限幅 ----
        vx_body = max(-self.vmax, min(self.vmax, vx_body))
        vy_body = max(-self.vmax, min(self.vmax, vy_body))
        wz_body = max(-self.wmax, min(self.wmax, wz_body))

        # ---- 发布 cmd_vel ----
        twist = Twist()
        twist.linear.x = vx_body
        twist.linear.y = vy_body
        twist.angular.z = wz_body
        self.cmd_pub.publish(twist)

        # ---- 发布 ctrl_debug ----
        dbg = CtrlDebug()
        dbg.header.stamp = now
        dbg.ex = debug['ex']
        dbg.ey = debug['ey']
        dbg.e_yaw = debug['e_yaw']
        dbg.cmd_raw_x = debug['vx_cmd_raw']
        dbg.cmd_raw_y = debug['vy_cmd_raw']
        dbg.cmd_raw_z = debug['wz_cmd_raw']
        self.debug_pub.publish(dbg)

    def publish_zero(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('controller_node')
    ControllerNode()
    rospy.spin()
