#!/usr/bin/env python3

import time
from typing import Callable, Union, Tuple
import numpy as np

class Pid:
    def __init__(
        self,
        pid_parameters: Callable[[str], Tuple[float, float, float, float]],
        setpoint: Callable[[str], Union[int, float]],
        max_output: Union[int, float],
        min_output: Union[int, float],
        type_motion: str  # "ThrottleSlave", "YawSlave", etc.
    ) -> None:

        self.pid_parameters = pid_parameters
        self.setpoint_func = setpoint
        self.type_motion = type_motion

        self.max_output = max_output
        self.min_output = min_output

        self.system_variable = 0.0
        self.setpoint_value = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.output = 0.0

        self.previous_time = None
        self.previous_error = 0.0

    def restart(self) -> None:
        self.integral = 0.0
        self.previous_time = None
        self.previous_error = 0.0

    def _get_speed(self, dt: float) -> float:
        kp, ki, kd, self.system_variable = self.pid_parameters(self.type_motion)

        self.error = self.setpoint_value - self.system_variable
        self.integral += self.error * dt
        derivative = (self.error - self.previous_error) / dt if dt > 0 else 0.0

        output = kp * self.error + ki * self.integral + kd * derivative

        # Anti-windup
        if output > self.max_output:
            output = self.max_output
            self.integral -= self.error * dt
        elif output < self.min_output:
            output = self.min_output
            self.integral -= self.error * dt

        self.previous_error = self.error
        return output

    def sequence(self) -> int:
        # Actualizar setpoint con manejo de errores
        try:
            self.setpoint_value = float(self.setpoint_func(self.type_motion))
            if self.type_motion == "ThrottleMaster" and self.setpoint_value <= 10:
                self.setpoint_value = 10
        except Exception:
            if self.type_motion == "ThrottleMaster":
                self.setpoint_value = 10.0
            else:
                self.setpoint_value = 0.0

        current_time = time.time()
        if self.previous_time is None:
            self.previous_time = current_time
            return 0

        dt = current_time - self.previous_time
        self.previous_time = current_time

        dt = max(dt, 1e-8)  # Evitar división por cero

        control_signal = self._get_speed(dt)
        self.output = int(np.clip(control_signal, self.min_output, self.max_output))
        return self.output

    def get_data(self) -> Tuple[float, float, float, float]:
        return self.system_variable, self.setpoint_value, self.error, self.output
