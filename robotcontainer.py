import robot
from components.autoAim import AutoAim
from components.drivetrain import DriveTrain
from components.physics import Physics
from components.state_handler import StateHandler
from components.vision import Vison
from components.hanger import Hanger
from components.shooter import shooter
from components.arm import Arm
#from auto.threeNote import ThreeNote

class RobotContainer:
   def __init__(self) -> None:
      self.drivetrain = DriveTrain()
      self.vision = Vison()
      #self.auto_aim = AutoAim()
      self.physics = Physics()
      self.StateHandler = StateHandler()
      #self.hanger = Hanger()
      #self.shooter = shooter()
      #self.arm = Arm()
      #self.ThreeNote = ThreeNote("nah id win", True, self.drivetrain)



   def getAutoCommand(self):
      #return self.drivetrain.getAutonomousCommand()
      pass