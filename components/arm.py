import phoenix5
import wpilib
import math
from wpimath import controller
import math

import phoenix5
import wpilib
from wpimath import controller


class Arm():

   def __init__(self):
      self.arm = phoenix5.TalonFX(0,"rio")
      self.driver = wpilib.Joystick(0)
      self.Kp = 0.001
      self.Ki = 0
      self.Kd = 0


      self.armPID = controller.PIDController(self.Kp, self.Ki, self.Kd)
      self.armPID.enableContinuousInput(-math.pi, math.pi)
      self.armPID.setSetpoint(0)

      if self.driver.getRawButtonPressed(5):
         self.arm.set(phoenix5.TalonFXControlMode.PercentOutput, 0.5)

