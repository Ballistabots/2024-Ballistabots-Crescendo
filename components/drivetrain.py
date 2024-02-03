import math

import wpilib
import wpimath.geometry
import wpimath.kinematics
from wpimath.geometry import Pose2d

import components.swervemodule as swervemodule

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
   def __init__(self) -> None:
      # sets locations of the modules
      self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
      self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
      self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
      self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

      # creates the motor objects
      self.frontLeft = swervemodule.SwerveModule(4, 0, 0, 3, 0, 0)
      self.frontRight = swervemodule.SwerveModule(6, 0, 0, 5, 0, 0)
      self.backLeft = swervemodule.SwerveModule(2, 0, 0, 1, 0, 0)
      self.backRight = swervemodule.SwerveModule(8, 0, 0, 7, 0, 0)

      self.gyro = wpilib.AnalogGyro(0)

      self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
         # makes kinematics happy
         self.frontLeftLocation,
         self.frontRightLocation,
         self.backLeftLocation,
         self.backRightLocation,
      )

      # odometry without odometry pods :(
      self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
         # used for path planning and auto
         self.kinematics,
         self.gyro.getRotation2d(),
         (
            self.frontLeft.GetPosition(),
            self.frontRight.GetPosition(),
            self.backLeft.GetPosition(),
            self.backRight.GetPosition(),
         ),
      )

      # self.gyro.reset()

   def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, periodSeconds: float, ) -> None:
      swerveModuleStates = self.kinematics.toSwerveModuleStates(
         wpimath.kinematics.ChassisSpeeds.discretize(
            wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
               xSpeed, ySpeed, rot, self.gyro.getRotation2d()
            )

            if fieldRelative
            else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds,
         )
      )
      wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed)
      self.frontLeft.SetState(swerveModuleStates[0])
      self.frontRight.SetState(swerveModuleStates[1])
      self.backLeft.SetState(swerveModuleStates[2])
      self.backRight.SetState(swerveModuleStates[3])

   def updateOdometry(self) -> Pose2d:
      # Updates the field relative position of the robot
      return self.odometry.update(
         self.gyro.getRotation2d(),
         (
            self.frontLeft.GetPosition(),
            self.frontRight.GetPosition(),
            self.backLeft.GetPosition(),
            self.backRight.GetPosition(),
         ),
      )

   def DriveTo(self, x: int, y: int, rot:float, speed: float = 0.5) -> str:
      """
      drives using encoders to cartesian coordinates
      """

      currentPos = self.updateOdometry()

      requestedX_Pose = currentPos.x + x
      requestedY_Pose = currentPos.y + y
      requestedRot_Pose = self.odometry.getPose() + rot
      return f"Driving to {x},{y}, pose is {rot} at speed {speed}"
   def Spline(self, x: int, y: int, rot:float):
      self.DriveTo(x, y, rot)