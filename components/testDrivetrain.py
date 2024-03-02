from wpimath.kinematics import ChassisSpeeds

import robotcontainer
from robotcontainer import SwerveModule
class testDrivetrain:
   def __init__(self):

      self.FrontLeftModule = SwerveModule(6,3,13)
      self.FrontRightModule = SwerveModule(5,2,10)
      self.BackLeftModule = SwerveModule(7,1,12)
      self.BackRightModule = SwerveModule(4,8,11)

      self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

      self.gyro = 1

