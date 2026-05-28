"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass, field
from typing import Optional

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float = 0.25
  v_ego_starting: float = 0.10
  stopping_decel_rate: float = 0.40
  lookahead_jerk_bp: list[float] = field(default_factory=lambda: [2., 5., 20.])
  lookahead_jerk_upper_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  lookahead_jerk_lower_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  longitudinal_actuator_delay: float = 0.50
  jerk_limits: float = 4.0

  # Optional per-car stop/launch refinements. When None the generic path is used.
  # Minimum accel hold during launch from standstill (speed -> min accel lookup).
  launch_hold_speed_bp: Optional[list[float]] = None
  launch_hold_speed_v: Optional[list[float]] = None
  # Jerk upper cap when restarting from a stop.
  stop_release_jerk_bp: Optional[list[float]] = None
  stop_release_jerk_v: Optional[list[float]] = None


# Default configurations for different car types
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(
    v_ego_stopping=0.30,
  ),
  "EV": CarTuningConfig(
    stopping_decel_rate=0.45,
    v_ego_stopping=0.35,
  ),
  "HYBRID": CarTuningConfig(
    v_ego_starting=0.15,
    stopping_decel_rate=0.45,
    v_ego_stopping=0.4,
  ),
  "DEFAULT": CarTuningConfig(
    v_ego_stopping=0.3,
  )
}

# Car-specific configs
_IONIQ_6_RESPONSE_MULTIPLIER = 1.2

CAR_SPECIFIC_CONFIGS = {
  CAR.HYUNDAI_IONIQ_6: CarTuningConfig(
    v_ego_stopping=0.30,
    # Jerk limits scaled by the Ioniq 6 response multiplier (1.2x).
    jerk_limits=4.8 * _IONIQ_6_RESPONSE_MULTIPLIER,
    # Shorter lookahead windows → more responsive jerk tracking.
    lookahead_jerk_upper_v=[0.3 / _IONIQ_6_RESPONSE_MULTIPLIER,
                            0.45 / _IONIQ_6_RESPONSE_MULTIPLIER,
                            0.6 / _IONIQ_6_RESPONSE_MULTIPLIER],
    lookahead_jerk_lower_v=[0.3 / _IONIQ_6_RESPONSE_MULTIPLIER,
                            0.45 / _IONIQ_6_RESPONSE_MULTIPLIER,
                            0.6 / _IONIQ_6_RESPONSE_MULTIPLIER],
    # Launch hold: minimum positive accel from standstill to 2.5 m/s.
    launch_hold_speed_bp=[0.0, 0.6, 1.25, 2.5],
    launch_hold_speed_v=[0.75, 0.6, 0.4, 0.0],
    # Stop release jerk cap: smooths the initial jerk when pulling away from rest.
    stop_release_jerk_bp=[0.0, 0.15, 0.5],
    stop_release_jerk_v=[3.6 * _IONIQ_6_RESPONSE_MULTIPLIER,
                         4.2 * _IONIQ_6_RESPONSE_MULTIPLIER,
                         4.8 * _IONIQ_6_RESPONSE_MULTIPLIER],
  ),
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.1,
    stopping_decel_rate=0.3,
    jerk_limits=3.3,
  ),
  CAR.KIA_NIRO_PHEV_2022: CarTuningConfig(
    stopping_decel_rate=0.8,
    jerk_limits=5.0,
  ),
}
