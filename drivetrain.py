import rev
import math

import wpilib
import wpimath
from wpimath import controller
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
   SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import phoenix6 as ctre
#from wpilib import DriverStation
import navx


def lratio(angle):
   """converts -pi, pi to -.5,.5"""
   return ((angle / math.pi) * -.5)


def ticks2rad(something):
   return (something / .5) * -math.pi


def deg2Rot2d(deg) -> Rotation2d:
   SwerveModulePosition()
   return Rotation2d(deg.value_as_double % 360 * (math.pi / 180))


def getSwerveModPos(rotEnc: ctre.hardware.CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
   return SwerveModulePosition(
      driveEnc.getPosition(),
      Rotation2d(ticks2rad(rotEnc.get_absolute_position().value_as_double))
   )


class DriveTrain():
   def __init__(self) -> None:
      super().__init__()

      self.backLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightRotation = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightRotation = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)


      self.backLeftDrive = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightDrive = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)

      self.frontRightDriveEnc = self.frontRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backRightDriveEnc = self.backRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backLeftDriveEnc = self.backLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

      self.BleftEnc = ctre.hardware.CANcoder(10)
      self.BrightEnc = ctre.hardware.CANcoder(13)
      self.FleftEnc = ctre.hardware.CANcoder(11)
      self.FrightEnc = ctre.hardware.CANcoder(12)

      self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

      Kp = 1.5
      Ki = 0
      Kd = 0
      self.BleftPID = controller.PIDController(Kp, Ki, Kd)
      self.BleftPID.enableContinuousInput(-math.pi, math.pi)
      self.BleftPID.setSetpoint(0.0)
      self.BrightPID = controller.PIDController(Kp, Ki, Kd)
      self.BrightPID.enableContinuousInput(-math.pi, math.pi)
      self.BrightPID.setSetpoint(0.0)
      self.FleftPID = controller.PIDController(Kp, Ki, Kd)
      self.FleftPID.enableContinuousInput(-math.pi, math.pi)
      self.FleftPID.setSetpoint(0.0)
      self.FrightPID = controller.PIDController(Kp, Ki, Kd)
      self.FrightPID.enableContinuousInput(-math.pi, math.pi)
      self.FrightPID.setSetpoint(0.0)

      self.gyro = navx.AHRS.create_i2c(wpilib.I2C.Port(0))
      self.gyro.enableLogging(True)


      frontrightlocation = Translation2d(.381, .381)
      frontleftlocation = Translation2d(.381, -.381)
      backleftlocation = Translation2d(-.381, -.381)
      backrightlocation = Translation2d(-.381, .381)

      self.kinematics = SwerveDrive4Kinematics(
         frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
      )

      self.odometry = SwerveDrive4Odometry(
         self.kinematics,
         wpimath.geometry.Rotation2d(self.gyro.getYaw())
        # wpimath.geometry.Rotation2d(0.0)
         ,
         (
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         )
      )


      print("end of init")

   def getAutonomousCommand(self):
      print("getAutocommand")
      # Load the path you want to follow using its name in the GUI

   def shouldFlipPath(self):
      pass
      # Boolean supplier that controls when the path will be mirrored for the red alliance
      # This will flip the path being followed to the red side of the field.
      # THE ORIGIN WILL REMAIN ON THE BLUE SIDE

     # return DriverStation.getAlliance() == DriverStation.Alliance.kRed

   def getChassisSpeed(self) -> ChassisSpeeds:
      print(f"{self.lastChassisSpeed=}")
      return self.lastChassisSpeed

   def updateOdometry(self) -> None:

      self.odometry.update(
         wpimath.geometry.Rotation2d(self.gyro.getYaw())
         ,
         (
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         )
      )

   def periodic(self) -> None:
      self.updateOdometry()

   def testDrive(self, speeds: ChassisSpeeds) -> None:
      print("TEST DRIVE")
      print(speeds)

   def testGetPose(self) -> Pose2d:
      print("getPOSE")
      return Pose2d()

   def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
      self.lastChassisSpeed = speeds
      frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

      frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
                                                      Rotation2d(
                                                         ticks2rad(self.FleftEnc.get_absolute_position()._value)))
      frontRightOptimized = SwerveModuleState.optimize(frontRight,
                                                       Rotation2d(
                                                          ticks2rad(self.FrightEnc.get_absolute_position()._value)))
      backLeftOptimized = SwerveModuleState.optimize(backLeft,
                                                     Rotation2d(
                                                        ticks2rad(self.BleftEnc.get_absolute_position()._value)))
      backRightOptimized = SwerveModuleState.optimize(backRight,
                                                      Rotation2d(
                                                         ticks2rad(self.BrightEnc.get_absolute_position()._value)))

      self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value,
                                                         lratio(backLeftOptimized.angle.radians())))
      self.frontLeftRotation.set(self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value,
                                                          lratio(frontLeftOptimized.angle.radians())))
      self.backRightRotation.set(self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value,
                                                           lratio(backRightOptimized.angle.radians())))
      self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value,
                                                            lratio(frontRightOptimized.angle.radians())))

      self.backLeftDrive.set(backLeftOptimized.speed)
      self.backRightDrive.set(backRightOptimized.speed)
      self.frontLeftDrive.set(frontLeftOptimized.speed)
      self.frontRightDrive.set(frontRightOptimized.speed)

      self.updateOdometry()


