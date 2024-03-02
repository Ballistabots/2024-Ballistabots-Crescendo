# from wpilib import DriverStation
import math

import phoenix6 as ctre
import rev
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState


class SwerveModule:
   def __init__(self, drive_can, turn_can, encoder_can):
      self.driveMotor = rev.CANSparkMax(drive_can, rev.CANSparkMax.MotorType.kBrushless)
      self.driveMotorEncoder = self.driveMotor.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

      self.turnMotor = rev.CANSparkMax(turn_can, rev.CANSparkMax.MotorType.kBrushless)

      self.encoder = ctre.hardware.CANcoder(encoder_can)

      self.turnKp = 2.5
      self.turnKd = 0.5

      self.turnPID = controller.PIDController(self.turnKp, 0, self.turnKd)
      self.turnPID.enableContinuousInput(-0.5, 0.5)
      self.turnPID.setSetpoint(0.0)

      self.driveKp = 0.001
      self.driveKi = 0
      self.driveKd = 0

      self.DrivePID = controller.ProfiledPIDController(self.driveKp, self.driveKi, self.driveKd,
                                                       wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

   def setSwivelDirection(self):
      pass

   def getRotEncPoseAsDouble(self):
      return self.encoder.get_absolute_position().value_as_double

   def getRotEncPose(self):
      return self.encoder.get_absolute_position().value

   def getDriveEncPosition(self):
      return self.driveMotorEncoder.getPosition()

   def getSwerveModulePose(self) -> SwerveModulePosition:
      return SwerveModulePosition(
         self.getDriveEncPosition(),
         Rotation2d(self.ticks2rad(self.getRotEncPoseAsDouble()))
      )

   def getAbsoluteEncoderRad(self) -> float:
      angle = self.getRotEncPoseAsDouble()
      angle *= 2 * math.pi
      return angle

   def getDriveVelocity(self):
      return self.driveMotorEncoder.getVelocity()

   def getState(self):
      return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getRotEncPoseAsDouble()))

   def setDesiredState(self, state: SwerveModuleState):
      self.state = SwerveModuleState.optimize(state, self.getState().angle)
      self.driveMotor.set(self.state.speed / 3)
      self.turnMotor.set(
         self.turnPID.calculate(self.getAbsoluteEncoderRad(), self.state.angle.radians()))

   def ticks2rad(self, something):
      """

      :param something: ticks
      :return: ur mom
      """
      return (something / .5) * -math.pi

   def setRotPower(self, power):
      self.turnMotor.set(power)
