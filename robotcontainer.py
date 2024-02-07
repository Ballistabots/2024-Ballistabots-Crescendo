from components.drivetrain import DriveTrain
from components.vision import Vison
from components.autoAim import AutoAim
from components.physics import Physics
import robot


class RobotContainer:
   def __init__(self) -> None:
      self.drivetrain = DriveTrain()
      self.vision = Vison()
      self.auto_aim = AutoAim()
      self.physics = Physics()
      self.robot = robot.MyRobot

   def getAutoCommand(self):
      return self.drivetrain.getAutonomousCommand()
