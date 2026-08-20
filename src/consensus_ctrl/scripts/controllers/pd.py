  #!/usr/bin/env python3
"""PDController - 占位 PD 控制律"""

import math
from .base import ControllerBase


class PDController(ControllerBase):
    def __init__(self, params):
        super().__init__(params)
        self.Kp = params.get('Kp', 1.5)
        self.Kd = params.get('Kd', 0.0)
        self.Kp_yaw = params.get('Kp_yaw', 1.5)
        self.last_err = None

    @staticmethod
    def wrap_angle(a):
        """把角度归一化到 (-pi, pi]"""
        return math.atan2(math.sin(a), math.cos(a))

    def compute(self, state, ref, dt):
        ex = ref['pos'][0] - state['x']
        ey = ref['pos'][1] - state['y']
        e_yaw = self.wrap_angle(ref['pos'][2] - state['yaw'])

        vx_cmd = ref['vel'][0] + self.Kp * ex
        vy_cmd = ref['vel'][1] + self.Kp * ey
        wz_cmd = ref['vel'][2] + self.Kp_yaw * e_yaw

        if self.last_err is not None and dt > 0:
            vx_cmd += self.Kd * (ex - self.last_err[0]) / dt
            vy_cmd += self.Kd * (ey - self.last_err[1]) / dt
        self.last_err = (ex, ey)

        debug = {
            'ex': ex,
            'ey': ey,
            'e_yaw': e_yaw,
            'vx_cmd_raw': vx_cmd,
            'vy_cmd_raw': vy_cmd,
            'wz_cmd_raw': wz_cmd,
        }
        return (vx_cmd, vy_cmd, wz_cmd), debug
