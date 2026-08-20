  #!/usr/bin/env python3
"""ControllerBase - 所有控制器的抽象基类"""


class ControllerBase:
    def __init__(self, params: dict):
        self.params = params

    def compute(self, state, ref, dt):
        """
        state: {x, y, yaw, vx, vy, wz}   世界系，来自 odom
        ref:   {pos, vel, acc}           参考轨迹
        dt:    实际时间步长

        返回:
          cmd:   (vx_cmd, vy_cmd, wz_cmd)  世界系速度指令
          debug: dict                      误差、内部状态等调试量
        """
        raise NotImplementedError("子类必须实现 compute()")
