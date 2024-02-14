
import phoenix6
import wpilib
from wpimath import controller
import robotcontainer


class Arm():

   def __init__(self):
      self.Joystick = wpilib.Joystick(0)
      self.state = robotcontainer.StateHandler
      self.driver = wpilib.Joystick(0)
      self.Kp = 0.001
      self.Ki = 0
      self.Kd = 0
      self.arm = phoenix6.hardware.TalonFX(0,"rio")

      self.armPID = controller.PIDController(self.Kp, self.Ki, self.Kd)
      self.armPID.enableContinuousInput(-0.5, 0.5)
      self.armPID.setSetpoint(0)



   def moveWithJoystickThrottle(self,throtle:float):
      self.arm.set_position(self.armPID.calculate(self.arm.get_position().value, throtle))

   def moveToPosition(self, newPos:float):
      self.arm.set_position(self.armPID.calculate(self.arm.get_position().value, newPos))

   def getArmPosition(self):
      pos = self.arm.get_position().value

      return pos
