import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, lkas_icon):
  values = {
    "LKA_OptUsmSta": 2,
    "LKA_SysIndReq": lkas_icon,
    "StrTqReqVal": apply_torque,
    "LKA_SysWrn": 0,
    "ActToiSta": 1 if lat_active else 0,
    "LKA_UsmMod": 0,  # hide LKAS settings
    "LKA_RcgSta": 0,
    "Damping_Gain": 100,  # can potentially tuned for better perf [3, 200]
  }

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # CAN FD camera-based SCC requires additional signals to be preserved
  # verbatim from the previous SCC_CONTROL frame to avoid checksum or
  # state validation faults. Classic CAN SCC only validates a subset.
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, lfa_icon):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control,
                       lead_data: CanFdLeadData, main_cruise_enabled, tuning):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel  # noqa: F841
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)  # noqa: F841

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1 if main_cruise_enabled else 0,
    "StopReq": 1 if tuning.stopping else 0,
    "aReqValue": tuning.actual_accel,
    "aReqRaw": tuning.actual_accel,
    "VSetDis": set_speed,
    "JerkLowerLimit": tuning.jerk_lower,
    "JerkUpperLimit": tuning.jerk_upper,

    "ACC_ObjDist": int(lead_data.lead_distance),
    "ACC_ObjRelSpd": lead_data.lead_rel_speed,
    "ObjValid": int(not lead_data.lead_visible),
    "SCC_ObjSta": 0 if not (enabled and lead_data.lead_visible) else (1 if gas_override else 2),
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(packer, CAN, frame):
  ret = []

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def create_adrv_messages(packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("ADRV_0x51", CAN.ACAN, values))

  ret.extend(create_fca_warning_light(packer, CAN, frame))

  if frame % 5 == 0:
    values = {
      'SET_ME_1C': 0x1c,
      'SET_ME_FF': 0xff,
      'SET_ME_TMP_F': 0xf,
      'SET_ME_TMP_F_2': 0xf,
    }
    ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    values = {
      'SET_ME_E1': 0xe1,
      'SET_ME_3A': 0x3a,
    }
    ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

  if frame % 20 == 0:
    values = {
      'SET_ME_15': 0x15,
    }
    ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

  if frame % 100 == 0:
    values = {
      'SET_ME_22': 0x22,
      'SET_ME_41': 0x41,
    }
    ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc


# *** Ioniq 6 cluster replay — restores BSM indicators and lane-change animations
# that the stock cluster normally receives from the ADAS ECU.  When OP longitudinal
# is active the ADAS ECU may be disabled, so we replay captured ECAN frames. ***

IONIQ_6_CLUSTER_BLINDSPOT_31A = {
  "right": (
    bytes.fromhex("fa7c10f0f0ffff03898aff0b0a8678ff000000007e0055550000000000000000"),
    bytes.fromhex("ac0e11f0f0ffff03898aff0c0a8678ff000000007e0055550000000000000000"),
    bytes.fromhex("76ce12f0f0ffff03898aff0b0a8678ff000000007e0055550000000000000000"),
    bytes.fromhex("309713f0f0ffff03898aff0b0a8678ff000000007e0055550000000000000000"),
    bytes.fromhex("d32214f0f0ffff03898aff0c0a8678ff000000007e0055550000000000000000"),
    bytes.fromhex("957b15f0f0ffff03898aff0c0a8678ff000000007e0055550000000000000000"),
  ),
  "left": (
    bytes.fromhex("851828f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("c34129f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("09aa2af0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("4ff32bf0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("bc6d2cf0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("fa342df0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
  ),
}

IONIQ_6_CLUSTER_BLINDSPOT_3B5 = {
  "right": (
    bytes.fromhex("caa95c00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("8cf05d00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("461b5e00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("00425f00000000464600000000000000d7020000000069070000000000000000"),
  ),
  "left": (
    bytes.fromhex("2c69c500000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("e682c600000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("21afc800000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("67f6c900000000464600000000000000da020000000069070000000000000000"),
  ),
}

IONIQ_6_CLUSTER_LANE_CHANGE_3C1 = {
  "right": {
    "trigger": bytes.fromhex("e910300041000000"),
    "steady":  bytes.fromhex("ab20300001000000"),
  },
  "left": {
    "trigger": bytes.fromhex("3d40304010000000"),
    "steady":  bytes.fromhex("3e50300000000000"),
  },
}

# Captured from a stock Ioniq 6 route showing the cluster lane-change animation
# on ECAN after the trigger/hold 0x3C1 states above.
IONIQ_6_CLUSTER_LANE_CHANGE_3B5 = {
  "right": (
    bytes.fromhex("9f687600000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("d9317700000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("58457800000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("1e1c7900000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("d4f77a00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("92ae7b00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("61307c00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("27697d00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("ed827e00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("abdb7f00000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("dd978000000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("9bce8100000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("51258200000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("177c8300000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("e4e28400000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("18ba8500000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("68508600000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("94088700000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("157c8800000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("53258900000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("99ce8a00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("df978b00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("2c098c00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("6a508d00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("a0bb8e00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("e6e28f00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("a2529000000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("e40b9100000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("2ee09200000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("d2b89300000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("9b279400000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("677f9500000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("17959600000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("ebcd9700000000464600000000000000d7020000000069070000000000000000"),
    bytes.fromhex("d0b89800000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("96e19900000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("5c0a9a00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("1a539b00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("e9cd9c00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("af949d00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("657f9e00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("23269f00000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("cc0fa000000000464600000000000000d8020000000069070000000000000000"),
    bytes.fromhex("bba6a100000000464600000000000000d9020000000069070000000000000000"),
    bytes.fromhex("714da200000000464600000000000000d9020000000069070000000000000000"),
    bytes.fromhex("3714a300000000464600000000000000d9020000000069070000000000000000"),
  ),
  "left": (
    bytes.fromhex("e682c600000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("d2dbc700000000464600000000000000d9020000000069070000000000000000"),
    bytes.fromhex("21afc800000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("67f6c900000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("ad1dca00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("eb44cb00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("18dacc00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("5e83cd00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("9468ce00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("d231cf00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("9681d000000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("d0d8d100000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("1a33d200000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("5c6ad300000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("ddf4d400000000464600000000000000d9020000000069070000000000000000"),
    bytes.fromhex("e9add500000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("2346d600000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("651fd700000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("e46bd800000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("d032d900000000464600000000000000d9020000000069070000000000000000"),
    bytes.fromhex("68d9da00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("2e80db00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("dd1edc00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("9b47dd00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("51acde00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("17f5df00000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("f8dce000000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("be85e100000000464600000000000000da020000000069070000000000000000"),
    bytes.fromhex("746ee200000000464600000000000000da020000000069070000000000000000"),
  ),
}

IONIQ_6_CLUSTER_LANE_CHANGE_31A = {
  "right": (
    bytes.fromhex("eb4518f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("757119f0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("bf9a1af0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("f9c31bf0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("0a5d1cf0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("4c041df0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("86ef1ef0f0ffff03898aff0a088678ff000000007e0055550000000000000000"),
    bytes.fromhex("18db1ff0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("f7f220f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
  ),
  "left": (
    bytes.fromhex("851828f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("c34129f0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("09aa2af0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("4ff32bf0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("bc6d2cf0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
    bytes.fromhex("fa342df0f0ffff03898aff0a098678ff000000007e0055550000000000000000"),
  ),
}

_IONIQ_6_CLUSTER_LANE_CHANGE_3C1_BURST = {
  0: "trigger",
  4: "trigger",
  7: "steady",
  10: "steady",
  13: "steady",
  16: "steady",
}
_IONIQ_6_LANE_CHANGE_3C1_STEADY_START = 34
_IONIQ_6_LANE_CHANGE_3B5_START = 4
_IONIQ_6_LANE_CHANGE_31A_START = 30


def create_blindspot_status_messages(packer, CAN, rear_values, front_corner_values,
                                     left_blindspot=False, right_blindspot=False,
                                     left_blinker=False, right_blinker=False):
  """Repack the last-known BLINDSPOTS_REAR_CORNERS / BLINDSPOTS_FRONT_CORNER_1 payloads
  with fresh rolling counter/checksum and updated indicator states.  Called when the
  originating ECU has stopped transmitting (stale timestamp)."""
  rear  = {k: v for k, v in rear_values.items() if k not in ("CHECKSUM", "COUNTER")}
  front = {k: v for k, v in front_corner_values.items() if k not in ("CHECKSUM", "COUNTER")}

  left_state  = 2 if left_blindspot and left_blinker else (1 if left_blindspot else 0)
  right_state = 2 if right_blindspot and right_blinker else (1 if right_blindspot else 0)

  rear["BCW_Sta"]           = int(left_blindspot or right_blindspot)
  rear["BCW_LtIndSta"]      = left_state
  rear["BCW_RtIndSta"]      = right_state
  rear["BCW_IndSta"]        = max(left_state, right_state)
  rear["OSMrrLamp_LtIndSta"] = left_state
  rear["OSMrrLamp_RtIndSta"] = right_state
  rear["FL_INDICATOR"]      = left_state
  rear["FR_INDICATOR"]      = right_state
  if "NEW_SIGNAL_3" not in front:
    front["NEW_SIGNAL_3"] = 1

  return [
    packer.make_can_msg("BLINDSPOTS_REAR_CORNERS",   CAN.ECAN, rear),
    packer.make_can_msg("BLINDSPOTS_FRONT_CORNER_1", CAN.ECAN, front),
  ]


def create_ioniq_6_cluster_blindspot_messages(CAN, frame, left_blindspot=False, right_blindspot=False,
                                              left_blinker=False, right_blinker=False):
  """Send cluster blindspot indicator frames on ECAN (0x3B5 @ 5 Hz, 0x31A @ 1 Hz)."""
  side = None
  if left_blindspot and not right_blindspot:
    side = "left"
  elif right_blindspot and not left_blindspot:
    side = "right"
  elif left_blindspot and right_blindspot:
    if left_blinker and not right_blinker:
      side = "left"
    elif right_blinker and not left_blinker:
      side = "right"

  if side is None:
    return []

  ret = []
  if frame % 20 == 0:
    seq = IONIQ_6_CLUSTER_BLINDSPOT_3B5[side]
    ret.append((0x3B5, seq[(frame // 20) % len(seq)], CAN.ECAN))
  if frame % 100 == 0:
    seq = IONIQ_6_CLUSTER_BLINDSPOT_31A[side]
    ret.append((0x31A, seq[(frame // 100) % len(seq)], CAN.ECAN))
  return ret


def create_ioniq_6_cluster_lane_change_messages(CAN, frame, side=None):
  """Send cluster lane-change animation frames on ECAN.
  frame counts up from 0 each time a new lane-change direction starts."""
  if side not in IONIQ_6_CLUSTER_LANE_CHANGE_3C1:
    return []

  ret = []
  frame_phase = _IONIQ_6_CLUSTER_LANE_CHANGE_3C1_BURST.get(frame)
  if frame_phase is None and frame >= _IONIQ_6_LANE_CHANGE_3C1_STEADY_START and \
     (frame - _IONIQ_6_LANE_CHANGE_3C1_STEADY_START) % 20 == 0:
    frame_phase = "steady"
  if frame_phase is not None:
    ret.append((0x3C1, IONIQ_6_CLUSTER_LANE_CHANGE_3C1[side][frame_phase], CAN.ECAN))

  if frame >= _IONIQ_6_LANE_CHANGE_3B5_START and (frame - _IONIQ_6_LANE_CHANGE_3B5_START) % 20 == 0:
    seq = IONIQ_6_CLUSTER_LANE_CHANGE_3B5[side]
    ret.append((0x3B5, seq[((frame - _IONIQ_6_LANE_CHANGE_3B5_START) // 20) % len(seq)], CAN.ECAN))
  if frame >= _IONIQ_6_LANE_CHANGE_31A_START and (frame - _IONIQ_6_LANE_CHANGE_31A_START) % 100 == 0:
    seq = IONIQ_6_CLUSTER_LANE_CHANGE_31A[side]
    ret.append((0x31A, seq[((frame - _IONIQ_6_LANE_CHANGE_31A_START) // 100) % len(seq)], CAN.ECAN))
  return ret
