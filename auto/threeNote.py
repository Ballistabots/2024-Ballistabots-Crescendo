from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds



class ThreeNote():
   def __init__(self, position: str, red: bool, drivetrain):
      self.DriveTrain = drivetrain
      self.speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Rotation2d(self.DriveTrain.getGyro()))
      self.maxSpeed = 0.5
      pass

   def ReversePath(self):
      pass

   def Aim(self):
      pass

   def StartFlyWheel(self):
      pass

   def StopFlyWheel(self):
      pass

   def Forward(self,backwards: bool = False) -> ChassisSpeeds:
      if backwards:
         return self.speeds.fromRobotRelativeSpeeds(-self.maxSpeed,0,0,self.DriveTrain.getGyro())
      else:
         return self.speeds.fromRobotRelativeSpeeds(self.maxSpeed,0,0,self.DriveTrain.getGyro())


   def Strafe(self, left: bool = False) -> ChassisSpeeds:
      if left:
         return self.speeds.fromRobotRelativeSpeeds(0,self.maxSpeed,0,self.DriveTrain.getGyro())
      else:
         return self.speeds.fromRobotRelativeSpeeds(0,-self.maxSpeed,0,self.DriveTrain.getGyro())


   def Turn(self, ccw: bool = False) -> ChassisSpeeds:
      if ccw:
         return self.speeds.fromRobotRelativeSpeeds(0,0,self.maxSpeed,self.DriveTrain.getGyro())
      else:
         return self.speeds.fromRobotRelativeSpeeds(0,0,-self.maxSpeed,self.DriveTrain.getGyro())


   def Path(self, currentTime: float) -> ChassisSpeeds:

      if currentTime < 2.5:
         return self.Forward(False)
      elif currentTime < 4:
         return self.Strafe(False)
      elif currentTime < 5:
         return self.Turn(False)


