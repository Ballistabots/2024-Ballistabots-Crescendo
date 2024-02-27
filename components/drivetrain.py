import math

# from wpilib import DriverStation
import navx
import phoenix6 as ctre
import rev
import wpilib
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
   SwerveModulePosition

import robotcontainer


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

      self.robotContainer = robotcontainer

      self.backLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightRotation = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightRotation = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)

      self.backLeftDrive = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightDrive = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)

      self.frontRightDriveEnc = self.frontRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backRightDriveEnc = self.backRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backLeftDriveEnc = self.backLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

      self.BleftEnc = ctre.hardware.CANcoder(12)
      self.BrightEnc = ctre.hardware.CANcoder(11)
      self.FleftEnc = ctre.hardware.CANcoder(13)
      self.FrightEnc = ctre.hardware.CANcoder(10)

      self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

      RotKp = 2.5  # 1.35
      RotKi = 0
      RotKd = 0.25
      self.BleftPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.BleftPID.enableContinuousInput(-math.pi, math.pi)
      self.BleftPID.setSetpoint(0.0)
      self.BrightPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.BrightPID.enableContinuousInput(-math.pi, math.pi)
      self.BrightPID.setSetpoint(0.0)
      self.FleftPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.FleftPID.enableContinuousInput(-math.pi, math.pi)
      self.FleftPID.setSetpoint(0.0)
      self.FrightPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.FrightPID.enableContinuousInput(-math.pi, math.pi)
      self.FrightPID.setSetpoint(0.0)

      # Drive Wheels Pid

      DriveKp = 1.5
      DriveKi = 0
      DriveKd = 0.15

      self.FrontRightDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                 wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.FrontLeftDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.BackRightDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.BackLeftDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                               wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.gyro = navx.AHRS.create_i2c(wpilib.I2C.Port.kMXP)

      self.gyro.enableLogging(True)

      frontrightlocation = Translation2d(.381, .381)
      frontleftlocation = Translation2d(.381, -.381)
      backleftlocation = Translation2d(-.381, -.381)
      backrightlocation = Translation2d(-.381, .381)

      self.kinematics = SwerveDrive4Kinematics(
         frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
      )
      yaw = -self.gyro.getAngle()
      h = yaw % 360  # formula to transform the yaw given by the gyro into a heading
      if h < 0:
         h += 360

      h2 = h / 360

      heading = h2 * (math.pi * 2)
      self.odometry = SwerveDrive4Odometry(
         self.kinematics,
         Rotation2d(heading)
         # wpimath.geometry.Rotation2d(0.0)
         ,
         (
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         )
         # Pose2d(5.0, 13.0, Rotation2d(0))
         # starting pose 5 meters against the wall 13.5 from the driver station and a heading of 0
      )

      print("end of init")

   def getAutonomousCommand(self):
      print("getAutocommand")
      # Load the path you want to follow using its name in the GUI

   def getPose(self):
      return self.odometry.getPose()

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
      yaw = -self.gyro.getAngle()
      h = yaw % 360  # formula to transform the yaw given by the gyro into a heading
      if h < 0:
         h += 360

      h2 = h / 360

      heading = h2 * (math.pi * 2)

      self.odometry.update(
         Rotation2d(heading)
         ,
         (
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         )
      )

   def resetOdometry(self, newPos: Pose2d) -> None:
      yaw = -self.gyro.getAngle()
      h = yaw % 360  # formula to transform the yaw given by the gyro into a heading
      if h < 0:
         h += 360

      h2 = h / 360

      heading = h2 * (math.pi * 2)

      self.odometry.resetPosition(
         Rotation2d(heading),
         modulePositions=(
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         ),
         pose=newPos
      )

   def periodic(self) -> None:
      self.updateOdometry()

   def testDrive(self, speeds: ChassisSpeeds) -> None:
      print("TEST DRIVE")
      print(speeds)

   def testGetPose(self) -> Pose2d:
      print("getPOSE")
      return Pose2d()

   def angleToEncTics(self, angle: float) -> float:

      return self.scale_number(angle, 0, 360, -0.5, 0.99973)

   def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
      """
      scales numbers using some cool math with other stuff

      """
      return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min

   def TurnSwivelPos(self):

      newAngle = self.angleToEncTics(45)

      FLnewState = self.optimize(0, newAngle, self.FleftEnc.get_absolute_position().value)
      FRnewState = self.optimize(0, 360 - newAngle, self.FrightEnc.get_absolute_position().value)
      BLnewState = self.optimize(0, 360 - newAngle, self.BleftEnc.get_absolute_position().value)
      BRnewState = self.optimize(0, newAngle, self.BrightEnc.get_absolute_position().value)

      FLnewSteerAngle = FLnewState[1]
      FRnewSteerAngle = FRnewState[1]
      BLnewSteerAngle = BLnewState[1]
      BRnewSteerAngle = BRnewState[1]

      FLOutput = self.FleftPID.calculate(self.FleftEnc.get_absolute_position().value, FLnewSteerAngle)
      FROutput = self.FrightPID.calculate(self.FrightEnc.get_absolute_position().value, FRnewSteerAngle)
      BLOutput = self.BleftPID.calculate(self.BleftEnc.get_absolute_position().value, BLnewSteerAngle)
      BROutput = self.BrightPID.calculate(self.BrightEnc.get_absolute_position().value, BRnewSteerAngle)

      self.frontLeftRotation.set(FLOutput)
      self.frontRightRotation.set(FROutput)
      self.backLeftRotation.set(BLOutput)
      self.backRightRotation.set(BROutput)

   def Defense(self, Enable: bool):
      if Enable == True:
         newAngle = self.angleToEncTics(230)

         FLnewState = self.optimize(0, newAngle, self.FleftEnc.get_absolute_position().value)
         FRnewState = self.optimize(0, 360 - newAngle, self.FrightEnc.get_absolute_position().value)
         BLnewState = self.optimize(0, 360 - newAngle, self.BleftEnc.get_absolute_position().value)
         BRnewState = self.optimize(0, newAngle, self.BrightEnc.get_absolute_position().value)

         FLnewSteerAngle = FLnewState[1]
         FRnewSteerAngle = FRnewState[1]
         BLnewSteerAngle = BLnewState[1]
         BRnewSteerAngle = BRnewState[1]

         FLOutput = self.FleftPID.calculate(self.FleftEnc.get_absolute_position().value, FLnewSteerAngle)
         FROutput = self.FrightPID.calculate(self.FrightEnc.get_absolute_position().value, FRnewSteerAngle)
         BLOutput = self.BleftPID.calculate(self.BleftEnc.get_absolute_position().value, BLnewSteerAngle)
         BROutput = self.BrightPID.calculate(self.BrightEnc.get_absolute_position().value, BRnewSteerAngle)

         self.frontLeftRotation.set(-FLOutput)
         self.frontRightRotation.set(-FROutput)
         self.backLeftRotation.set(BLOutput)
         self.backRightRotation.set(-BROutput)

   def SetSwivelDirection(self, angle):
      # sets the new requested states of the swivels

      newAngle = self.angleToEncTics(angle)

      FLnewState = self.optimize(0, newAngle, self.FleftEnc.get_absolute_position().value)
      FRnewState = self.optimize(0, newAngle, self.FrightEnc.get_absolute_position().value)
      BLnewState = self.optimize(0, newAngle, self.BleftEnc.get_absolute_position().value)
      BRnewState = self.optimize(0, newAngle, self.BrightEnc.get_absolute_position().value)

      FLnewSteerAngle = FLnewState[1]
      FRnewSteerAngle = FRnewState[1]
      BLnewSteerAngle = BLnewState[1]
      BRnewSteerAngle = BRnewState[1]

      FLOutput = self.FleftPID.calculate(self.FleftEnc.get_absolute_position().value, FLnewSteerAngle)
      FROutput = self.FrightPID.calculate(self.FrightEnc.get_absolute_position().value, FRnewSteerAngle)
      BLOutput = self.BleftPID.calculate(self.BleftEnc.get_absolute_position().value, BLnewSteerAngle)
      BROutput = self.BrightPID.calculate(self.BrightEnc.get_absolute_position().value, BRnewSteerAngle)

      self.frontLeftRotation.set(FLOutput)
      self.frontRightRotation.set(FROutput)
      self.backLeftRotation.set(BLOutput)
      self.backRightRotation.set(BROutput)

   def optimize(self, drive_voltage, steer_angle, current_angle):
      delta = steer_angle - current_angle

      if abs(delta) > math.pi / 2.0 and abs(delta) < 3.0 / 2.0 * math.pi:
         if steer_angle >= math.pi:
            return (-drive_voltage, steer_angle - math.pi)
         else:
            return (-drive_voltage, steer_angle + math.pi)
      else:
         return (drive_voltage, steer_angle)

   def setSwivel(self, desiredState: wpimath.kinematics.ChassisSpeeds) -> None:
      """
       SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
      :return:
      """

      self.states = wpimath.kinematics.SwerveDrive4Kinematics
      self.states.toSwerveModuleStates()

   def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
      self.lastChassisSpeed = speeds
      frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

      frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
                                                      Rotation2d(
                                                         ticks2rad(
                                                            self.FleftEnc.get_absolute_position().value_as_double)))
      frontRightOptimized = SwerveModuleState.optimize(frontRight,
                                                       Rotation2d(
                                                          ticks2rad(
                                                             self.FrightEnc.get_absolute_position().value_as_double)))
      backLeftOptimized = SwerveModuleState.optimize(backLeft,
                                                     Rotation2d(
                                                        ticks2rad(
                                                           self.BleftEnc.get_absolute_position().value_as_double)))
      backRightOptimized = SwerveModuleState.optimize(backRight,
                                                      Rotation2d(
                                                         ticks2rad(
                                                            self.BrightEnc.get_absolute_position().value_as_double)))

      self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position().value_as_double,
                                                         lratio(backLeftOptimized.angle.radians())))
      self.frontLeftRotation.set(self.FleftPID.calculate(self.FleftEnc.get_absolute_position().value_as_double,
                                                         lratio(frontLeftOptimized.angle.radians())))
      self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position().value_as_double,
                                                           lratio(backRightOptimized.angle.radians())))
      self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position().value_as_double,
                                                            lratio(frontRightOptimized.angle.radians())))

      self.backLeftDrive.set(-backLeftOptimized.speed)
      self.backRightDrive.set(backRightOptimized.speed)
      self.frontLeftDrive.set(frontLeftOptimized.speed)
      self.frontRightDrive.set(frontRightOptimized.speed)

      self.updateOdometry()
