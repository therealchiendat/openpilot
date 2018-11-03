#!/usr/bin/env python
from cereal import car
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.mazda.values import DBC, CAR
from selfdrive.car.mazda.carstate import CarState, get_powertrain_can_parser

try:
  from selfdrive.car.mazda.carcontroller import CarController
except ImportError:
  CarController = None


class CanBus(object):
  def __init__(self):
    self.powertrain = 0
    self.obstacle = 1

class CarInterface(object):
  def __init__(self, CP, sendcan=None):
    self.CP = CP

    self.frame = 0
    self.can_invalid_count = 0
    self.acc_active_prev = 0

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)
    self.pt_cp = get_powertrain_can_parser(CP, canbus)

    # sending if read only is False
    if sendcan is not None:
      self.sendcan = sendcan
      self.CC = CarController(canbus, CP.carFingerprint)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint):
    ret = car.CarParams.new_message()

    ret.carName = "mazda"
    ret.carFingerprint = candidate

    ret.enableCruise = False

    # TODO: gate this on detection
    ret.enableCamera = True

    std_cargo = 136
    # hardcoding honda civic 2016 touring params so they can be used to
    # scale unknown params for other cars
    mass_civic = 2923./2.205 + std_cargo
    wheelbase_civic = 2.70
    centerToFront_civic = wheelbase_civic * 0.4
    centerToRear_civic = wheelbase_civic - centerToFront_civic
    rotationalInertia_civic = 2500
    tireStiffnessFront_civic = 192150
    tireStiffnessRear_civic = 202500

    if candidate in [CAR.CX5]:
      stop_and_go = True
      ret.mass =  3655 * CV.LB_TO_KG + std_cargo
      ret.wheelbase = 2.7
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 15.5
      ret.steerKf = 0.00006
      ret.steerKiBP, ret.steerKpBP = [[0.], [0.]]
      ret.steerKpV, ret.steerKiV = [[0.6], [0.1]]
      ret.steerMaxBP = [0.] # m/s
      ret.steerMaxV = [1.]
      tire_stiffness_factor = 0.70

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.5

    ret.safetyModel = car.CarParams.SafetyModels.mazda
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitAlert = False
    # testing tuning

    # FIXME: from gm
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]
    ret.brakeMaxBP = [5., 20.]
    ret.brakeMaxV = [1., 0.8]

    ret.longPidDeadzoneBP = [0.]
    ret.longPidDeadzoneV = [0.]

    ret.longitudinalKpBP = [5., 35.]
    ret.longitudinalKpV = [2.4, 1.5]
    ret.longitudinalKiBP = [0.]
    ret.longitudinalKiV = [0.36]

    ret.stoppingControl = True
    ret.startAccel = 0.8
    # end from gm

    centerToRear = ret.wheelbase - ret.centerToFront
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = rotationalInertia_civic * \
                            ret.mass * ret.wheelbase**2 / (mass_civic * wheelbase_civic**2)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront = tireStiffnessFront_civic * \
                             ret.mass / mass_civic * \
                             (centerToRear / ret.wheelbase) / (centerToRear_civic / wheelbase_civic)
    ret.tireStiffnessRear = tireStiffnessRear_civic * \
                            ret.mass / mass_civic * \
                            (ret.centerToFront / ret.wheelbase) / (centerToFront_civic / wheelbase_civic)

    return ret

  # returns a car.CarState
  def update(self, c):

    self.pt_cp.update(int(sec_since_boot() * 1e9), False)
    self.CS.update(self.pt_cp)

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    #ret.steeringRate = self.CS.angle_steers_rate

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    #ret.steeringTorque = self.CS.steer_torque_driver

    # cruise state
    ret.cruiseState.available = bool(self.CS.main_on)
    ret.leftBlinker = self.CS.left_blinker_on
    ret.rightBlinker = self.CS.right_blinker_on


    buttonEvents = []

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on
      buttonEvents.append(be)

    be = car.CarState.ButtonEvent.new_message()
    be.type = 'accelCruise'
    buttonEvents.append(be)


    events = []
    if not self.CS.can_valid:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 5:
        events.append(create_event('commIssue', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    else:
      self.can_invalid_count = 0

    if self.CS.acc_active and not self.acc_active_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    if not self.CS.acc_active:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
      if b.type in ["accelCruise", "decelCruise"] and not b.pressed:
        events.append(create_event('buttonEnable', [ET.ENABLE]))
      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
		
    ret.events = events

    # update previous brake/gas pressed
    self.acc_active_prev = self.CS.acc_active


    # cast to reader so it can't be modified
    return ret.as_reader()

  def apply(self, c, perception_state):
    self.CC.update(self.sendcan, c.enabled, self.CS, self.frame, c.actuators)
    self.frame += 1
