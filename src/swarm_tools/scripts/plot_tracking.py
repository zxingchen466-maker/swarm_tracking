#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_tracking.py —— 把 rosbag 画成三张验收图 (PDF)  [阶段4 v4]

用法:
    python3 plot_tracking.py <xxx.bag>

输出 (保存在 bag 同目录):
    1_trajectory.pdf      参考轨迹(虚线) vs 实际轨迹(实线), 标记起点, 含局部放大+间隙标注
    2_tracking_error.pdf  跟踪误差 e_x / e_y / e_yaw 随时间变化 (3个子图)
    3_control_input.pdf   控制量 vx/vy/wz: 原始输出、实际下发 + 饱和限幅线

注意:
    - 需要先 source 工作空间, 否则读不到 TrajRef/CtrlDebug 消息类型
    - 图上标签用英文: 虚拟机里可能没有中文字体, 中文会显示成方框
"""
import os
import sys
import argparse
import statistics

import matplotlib
matplotlib.use('Agg')   # 不弹窗口, 直接存文件 (适合脚本批量出图)
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

# ---------- 配色: 颜色按"身份"固定, 不按出场顺序轮换 ----------
C_REF  = '#2a78d6'   # 蓝: 参考/期望 (参考轨迹、控制器原始输出)
C_ACT  = '#eb6834'   # 橙: 实际 (实际轨迹、实际下发的指令)
C_EYAW = '#1baf7a'   # 青: 第三组数据 e_yaw
C_LIM  = '#d03b3b'   # 红: 饱和限幅线 (参考线, 不是数据)
C_GRID = '#e1e0d9'   # 浅灰: 网格线

FS = 12   # 基本字号 (任务要求 >= 12)
LW = 2.0  # 线宽   (任务要求 >= 1.5)
MS = 9    # 标记大小

TOPIC_ODOM = '/robot_1/odom'
TOPIC_REF  = '/robot_1/ref'
TOPIC_CMD  = '/robot_1/cmd_vel'
TOPIC_DBG  = '/robot_1/ctrl_debug'

EMPTY_MSG = {
    't_odom': TOPIC_ODOM,
    't_ref':  TOPIC_REF,
    't_cmd':  TOPIC_CMD,
    't_dbg':  TOPIC_DBG,
}


def load_bag(bag_path):
    """读取 bag: 4 个话题的消息按记录时间存成列表"""
    data = {
        't_odom': [], 'x_act': [], 'y_act': [],
        't_ref':  [], 'x_ref': [], 'y_ref': [],
        't_cmd':  [], 'cmd_x': [], 'cmd_y': [], 'cmd_z': [],
        't_dbg':  [], 'raw_x': [], 'raw_y': [], 'raw_z': [],
        'ex': [], 'ey': [], 'e_yaw': [],
    }
    bag = rosbag.Bag(bag_path, 'r')
    t0 = None
    for topic, msg, t in bag.read_messages():
        if t0 is None:
            t0 = t.to_sec()          # 以 bag 里第一条消息为时间零点
        ts = t.to_sec() - t0
        if topic == TOPIC_ODOM:
            data['t_odom'].append(ts)
            data['x_act'].append(msg.pose.pose.position.x)
            data['y_act'].append(msg.pose.pose.position.y)
        elif topic == TOPIC_REF:
            data['t_ref'].append(ts)
            data['x_ref'].append(msg.pos.x)
            data['y_ref'].append(msg.pos.y)
        elif topic == TOPIC_CMD:
            data['t_cmd'].append(ts)
            data['cmd_x'].append(msg.linear.x)
            data['cmd_y'].append(msg.linear.y)
            data['cmd_z'].append(msg.angular.z)
        elif topic == TOPIC_DBG:
            data['t_dbg'].append(ts)
            data['ex'].append(msg.ex)
            data['ey'].append(msg.ey)
            data['e_yaw'].append(msg.e_yaw)
            data['raw_x'].append(msg.cmd_raw_x)
            data['raw_y'].append(msg.cmd_raw_y)
            data['raw_z'].append(msg.cmd_raw_z)
    bag.close()

    # 话题缺失时给出清晰报错, 而不是后面画图时才崩
    for key, topic in EMPTY_MSG.items():
        if not data[key]:
            sys.exit('错误: bag 里没有话题 %s (key=%s), 请检查录制内容' % (topic, key))
    return data


def style_ax(ax):
    """统一样式: 浅灰网格, 弱化的轴线"""
    ax.grid(True, color=C_GRID, lw=0.8)
    ax.tick_params(labelsize=FS)
    for spine in ax.spines.values():
        spine.set_color('#898781')


def plot_trajectory(data, out_path):
    """图1: XY 平面 参考 vs 实际, 标记起点, 附局部放大+间隙标注"""
    fig, ax = plt.subplots(figsize=(7, 6))
    style_ax(ax)
    ax.plot(data['x_ref'], data['y_ref'], '--', color=C_REF, lw=LW,
            label='Reference (ref)')
    ax.plot(data['x_act'], data['y_act'], '-', color=C_ACT, lw=LW,
            label='Actual (odom)')
    ax.plot(data['x_act'][0], data['y_act'][0], 'o', color=C_ACT,
            ms=MS + 2, label='Start point')
    ax.set_xlabel('x [m]', fontsize=FS + 1)
    ax.set_ylabel('y [m]', fontsize=FS + 1)
    ax.set_title('XY Trajectory: Reference vs Actual', fontsize=FS + 2)
    ax.axis('equal')
    ax.legend(fontsize=FS, loc='best')

    # 局部放大镜: 取时间中段 1 秒, 放大后能看清两曲线间的毫米级间隙
    t_end = min(data['t_odom'][-1], data['t_ref'][-1])
    t_win = min(1.0, t_end * 0.2)
    t_lo = t_end * 0.5
    t_hi = t_lo + t_win
    t_act = np.array(data['t_odom'])
    t_ref = np.array(data['t_ref'])
    m_act = (t_act >= t_lo) & (t_act <= t_hi)
    m_ref = (t_ref >= t_lo) & (t_ref <= t_hi)
    xa_w = np.array(data['x_act'])[m_act]
    ya_w = np.array(data['y_act'])[m_act]
    xr_w = np.array(data['x_ref'])[m_ref]
    yr_w = np.array(data['y_ref'])[m_ref]

    axins = inset_axes(ax, width='42%', height='42%', loc='upper left')
    axins.plot(xr_w, yr_w, '--', color=C_REF, lw=LW * 0.7)
    axins.plot(xa_w, ya_w, '-', color=C_ACT, lw=LW * 0.7)

    # 在窗口内找最大间隙并标注: 每个实际点到参考曲线的最短距离
    d2 = (xa_w[:, None] - xr_w[None, :]) ** 2 + (ya_w[:, None] - yr_w[None, :]) ** 2
    dmin = np.sqrt(d2.min(axis=1))
    i = int(dmin.argmax())
    j = int(d2[i].argmin())
    axins.annotate('gap = %.1f mm' % (dmin[i] * 1000.0),
                   xy=(0.5 * (xa_w[i] + xr_w[j]), 0.5 * (ya_w[i] + yr_w[j])),
                   xytext=(0.02, 0.86), textcoords='axes fraction',
                   fontsize=FS, color='#0b0b0b',
                   arrowprops=dict(arrowstyle='->', color='#52514e', lw=1.0))
    axins.set_title('Zoom (middle %.1f s)' % t_win, fontsize=FS)
    axins.axis('equal')
    style_ax(axins)

    # 用 bbox_inches='tight' 保存: 按所有元素(含放大镜)自动收边距, 无排版警告
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print('图1 已保存:', out_path)


def plot_errors(data, out_path):
    """图2: 三个子图, e_x / e_y / e_yaw 随时间变化"""
    fig, axes = plt.subplots(3, 1, figsize=(7, 9), sharex=True)
    items = [
        ('e_x',   data['ex'],    'e_x [m]',     C_REF),
        ('e_y',   data['ey'],    'e_y [m]',     C_ACT),
        ('e_yaw', data['e_yaw'], 'e_yaw [rad]', C_EYAW),
    ]
    for ax, (name, val, ylab, color) in zip(axes, items):
        style_ax(ax)
        ax.plot(data['t_dbg'], val, color=color, lw=LW)
        ax.set_ylabel(ylab, fontsize=FS + 1)
        # 单条曲线不需要图例框, 直接把名字标在曲线旁边
        ax.text(0.985, 0.94, name, transform=ax.transAxes,
                ha='right', va='top', fontsize=FS + 1, color=color)
    axes[-1].set_xlabel('t [s]', fontsize=FS + 1)
    fig.suptitle('Tracking Error over Time', fontsize=FS + 2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print('图2 已保存:', out_path)


def plot_control(data, out_path):
    """图3: 三个子图, vx/vy/wz 原始输出 vs 实际下发 + 饱和限幅线"""
    fig, axes = plt.subplots(3, 1, figsize=(7, 9), sharex=True)
    items = [
        ('vx', data['raw_x'], data['cmd_x'], 'vx [m/s]',   1.0, '1.0 m/s'),
        ('vy', data['raw_y'], data['cmd_y'], 'vy [m/s]',   1.0, '1.0 m/s'),
        ('wz', data['raw_z'], data['cmd_z'], 'wz [rad/s]', 2.0, '2.0 rad/s'),
    ]
    for ax, (name, raw, cmd, ylab, vmax, lim) in zip(axes, items):
        style_ax(ax)
        ax.plot(data['t_dbg'], raw, color=C_REF, lw=LW,
                label='Raw output (before saturation)')
        ax.plot(data['t_cmd'], cmd, color=C_ACT, lw=LW,
                label='Sent command (after saturation)')
        ax.axhline(vmax, color=C_LIM, ls='--', lw=LW * 0.8,
                   label='Saturation limit ' + lim)
        ax.axhline(-vmax, color=C_LIM, ls='--', lw=LW * 0.8)
        ax.set_ylabel(ylab, fontsize=FS + 1)
        ax.legend(fontsize=FS, loc='upper right', framealpha=0.9)
    axes[-1].set_xlabel('t [s]', fontsize=FS + 1)
    fig.suptitle('Control Inputs with Saturation Limits', fontsize=FS + 2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print('图3 已保存:', out_path)


def main():
    parser = argparse.ArgumentParser(description='rosbag -> 3 张验收图 (PDF)')
    parser.add_argument('bag', help='bag 文件路径, 如 bags/xxx.bag')
    args = parser.parse_args()
    if not os.path.isfile(args.bag):
        sys.exit('找不到文件: ' + args.bag)

    print('正在读取:', args.bag)
    data = load_bag(args.bag)

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    plot_trajectory(data, os.path.join(out_dir, '1_trajectory.pdf'))
    plot_errors(data, os.path.join(out_dir, '2_tracking_error.pdf'))
    plot_control(data, os.path.join(out_dir, '3_control_input.pdf'))

    print('---------------- 误差统计 ----------------')
    for name, val in [('e_x', data['ex']), ('e_y', data['ey']),
                      ('e_yaw', data['e_yaw'])]:
        print('%-6s 均值 %.4f   最大绝对值 %.4f'
              % (name, statistics.mean(val), max(abs(v) for v in val)))


if __name__ == '__main__':
    main()
