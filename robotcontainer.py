from components.arm import Arm
from components.drivetrain import DriveTrain
from components.shooter import shooter


class RobotContainer:
   def __init__(self) -> None:
      self.drivetrain = DriveTrain()
      # self.vision = Vison()
      # self.auto_aim = AutoAim()
      # self.physics = Physics()
      # self.StateHandler = StateHandler()
      # self.hanger = Hanger()
      #self.shooter = shooter()
      self.arm = Arm()
      # self.path_test = path_test.PathTest


   def getAutoCommand(self):
      # return self.drivetrain.getAutonomousCommand()
      pass
