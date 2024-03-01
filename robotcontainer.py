from components.arm import Arm
from components.drivetrain import DriveTrain
from components.physics import Physics
from components.shooter import shooter
from components.state_handler import StateHandler
from components.vision import Vison
from auto import path_test
import robot


class RobotContainer:
   def __init__(self) -> None:
      self.drivetrain = DriveTrain()
      self.vision = Vison()
      # self.auto_aim = AutoAim()
      self.physics = Physics()
      self.StateHandler = StateHandler()
      # self.hanger = Hanger()
      self.shooter = shooter()
      self.arm = Arm
      self.path_test = path_test.PathTest
      self.robot = robot

   def getAutoCommand(self):
      # return self.drivetrain.getAutonomousCommand()
      pass
