import rev
import wpilib
import wpimath
import math
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.kinematics
import wpimath.trajectory
import wpimath.geometry

kWheelRadius = 2  # wheel radius
kEncoderResolution = 4096  # encoder resolution change once u know which encoders we using
kModuleMaxAngularVelocity = math.pi  # max angular vel for a single module#
kModuleMaxAngularAcceleration = math.tau  # max angular accel for a single module


class SwerveModule:

   def __init__(self, DriveMotorPort, DriveMotorEncoderA, DriveMotorEncoderB, TurnMotorPort, TurnMotorEncoderA,
                TurnMotorEncoderB) -> None:
      print(f"initilized {self}")
      # DriveMotorPort is the can port that has the driving motor
      # DriveMotorEncoderA is the dio input for the drive motor A
      # DriveMotorEncoderB is the dio input for the drive motor B

      # TurnMotorPort is the can port that has the driving motor
      # TurnMotorEncoderA is the dio input for the turning motor A
      # TurnMotorEncoderB is the dio input for the turning motor B

      self.DriveMotor = rev.CANSparkMax(DriveMotorPort,
                                        rev.CANSparkMax.MotorType.kBrushless)  # inits the drive motor as a brushless motor
      self.TurningMotor = rev.CANSparkMax(TurnMotorPort,
                                          rev.CANSparkMax.MotorType.kBrushless)  # inits the turning motor as a brushless motor

      self.DriveEncoder = wpilib.Encoder(DriveMotorEncoderA, DriveMotorEncoderB)
      self.TurnEncoder = wpilib.Encoder(TurnMotorEncoderA, TurnMotorEncoderB)

      self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)  # ready to tune will go very slow lollololl

      self.turningPIDController = wpimath.controller.ProfiledPIDController(
         # also ready to tune and will go very slow lolololololololol
         1,
         0,
         0,
         wpimath.trajectory.TrapezoidProfile.Constraints(
            kModuleMaxAngularVelocity,
            kModuleMaxAngularAcceleration,
         ),
      )

      self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
      self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

      self.DriveEncoder.setDistancePerPulse(
         math.tau * kWheelRadius / kEncoderResolution
      )

      self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

      # limits the input range between pi and -pi aswell as enabling coutnious input

   def GetState(self) -> wpimath.kinematics.SwerveModuleState:
      # returns the state the current module is in
      print(f"Returning state of {self.DriveEncoder}")
      return wpimath.kinematics.SwerveModuleState(
         self.DriveEncoder.getRate(),
         wpimath.geometry.Rotation2d(self.TurnEncoder.getDistance()),
      )

   def GetPosition(self) -> wpimath.kinematics.SwerveModulePosition:
      print(f"Returning position of {self.DriveEncoder}")
      # returns current position of the module
      return wpimath.kinematics.SwerveModulePosition(
         self.DriveEncoder.getRate(),
         wpimath.geometry.Rotation2d(self.TurnEncoder.getDistance()),
      )

   def SetState(self, desiredState: wpimath) -> None:
      encoderRotation = wpimath.geometry.Rotation2d(self.TurnEncoder.getDistance())

      # Optimize the reference state to avoid spinning further than 90 degrees
      state = wpimath.kinematics.SwerveModuleState.optimize(
         desiredState, encoderRotation
      )
      print(f"Setting state to {self.DriveEncoder}")
      # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
      # direction of travel that can occur when modules change directions. This results in smoother
      # driving.
      state.speed *= (state.angle - encoderRotation).cos()

      # Calculate the drive output from the drive PID controller.
      driveOutput = self.drivePIDController.calculate(
         self.DriveEncoder.getRate(), state.speed
      )

      driveFeedforward = self.driveFeedforward.calculate(state.speed)

      # Calculate the turning motor output from the turning PID controller.
      turnOutput = self.turningPIDController.calculate(
         self.TurnEncoder.getDistance(), state.angle.radians()
      )

      turnFeedforward = self.turnFeedforward.calculate(
         self.turningPIDController.getSetpoint().velocity
      )
      self.DriveEncoder.setVoltage(driveOutput + driveFeedforward)
      self.TurningMotor.setVoltage(turnOutput + turnFeedforward)
