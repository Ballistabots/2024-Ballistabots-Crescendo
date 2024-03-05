from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds


class ThreeNote():
   def __init__(self, position: str, red: bool, drivetrain):
      self.DriveTrain = drivetrain
      self.speeds = ChassisSpeeds#.fromRobotRelativeSpeeds(0, 0, 0, Rotation2d().fromDegrees(self.DriveTrain.getGyro()))
      self.maxSpeed = 0.3
      pass

   def ReversePath(self):
      pass

   def Aim(self):
      pass

   def StartFlyWheel(self):
      pass

   def StopFlyWheel(self):
      pass

   def Forward(self, backwards: bool = False) -> ChassisSpeeds:
      if backwards:
         return self.speeds.fromRobotRelativeSpeeds(self.maxSpeed, 0, 0,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))
      else:
         return self.speeds.fromRobotRelativeSpeeds(-self.maxSpeed, 0, 0,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))

   def Strafe(self, left: bool = False) -> ChassisSpeeds:
      if left:
         return self.speeds.fromRobotRelativeSpeeds(0, self.maxSpeed, 0,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))
      else:
         return self.speeds.fromRobotRelativeSpeeds(0, -self.maxSpeed, 0,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))

   def Turn(self, ccw: bool = False) -> ChassisSpeeds:
      if ccw:
         return self.speeds.fromRobotRelativeSpeeds(0, 0, self.maxSpeed,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))
      else:
         return self.speeds.fromRobotRelativeSpeeds(0, 0, -self.maxSpeed,
                                                    Rotation2d().fromDegrees(self.DriveTrain.getGyro()))

   def STOP(self):
      speeds =  self.speeds.fromRobotRelativeSpeeds(0, 0, 0, Rotation2d().fromDegrees(self.DriveTrain.getGyro()))
      self.DriveTrain.driveFromChassisSpeeds(speeds)

   def Path(self, currentTime: float) -> ChassisSpeeds:
      if currentTime < 0.5:
         return self.Forward()
      elif currentTime > 0.5:
         self.STOP()
