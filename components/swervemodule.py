import math
import wpilib
import rev

import wpimath
import wpimath.controller
import wpimath.filter
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import navx

kWheelRadius = 2  # wheel radius
kEncoderResolution = 4096  # encoder resolution
kModuleMaxAngularVelocity = math.pi  # max angular vel for a single module#
kModuleMaxAngularAcceleration = math.tau  # max angular accel for a single module


class SwerveModule:

   def __init__(self, DriveMotorPort, DriveMotorEncoder, TurnMotorPort) -> None:
      print(f"initilized {self}")
      # DriveMotorPort is the can port that has the driving motor
      # DriveMotorEncoderA is the dio input for the drive motor A
      # DriveMotorEncoderB is the dio input for the drive motor B

      # TurnMotorPort is the can port that has the driving motor
      # TurnMotorEncoderA is the dio input for the turning motor A
      # TurnMotorEncoderB is the dio input for the turning motor B
      self.gyro = navx.AHRS.create_i2c()

      self.DriveMotor = rev.CANSparkMax(DriveMotorPort,
                                        rev.CANSparkMax.MotorType.kBrushless)  # inits the drive motor as a brushless motor
      self.TurningMotor = rev.CANSparkMax(TurnMotorPort,
                                          rev.CANSparkMax.MotorType.kBrushless)  # inits the turning motor as a brushless motor

      self.DriveEncoder = wpilib.AnalogEncoder(DriveMotorEncoder)
      #self.TurnEncoder = wpilib.Encoder(TurnMotorEncoder)



      self.drivePIDController = wpimath.controller.PIDController(3, 0, 0.5)  # ready to tune will go very slow lollololl

      self.turningPIDController = wpimath.controller.ProfiledPIDController(
         # also ready to tune and will go very slow lolololololololol
         3,
         0,
         0.5,
         wpimath.trajectory.TrapezoidProfile.Constraints(
            kModuleMaxAngularVelocity,
            kModuleMaxAngularAcceleration,
         ),
      )

      self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
      self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

      #self.DriveEncoder.setDistancePerPulse(math.tau * kWheelRadius / kEncoderResolution)

      self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

      # limits the input range between pi and -pi aswell as enabling coutnious input

   def GetState(self) -> wpimath.kinematics.SwerveModuleState:
      #returns the state the current module is in
      print(f"Returning state of {self.DriveEncoder}")
      return wpimath.kinematics.SwerveModuleState(
         self.DriveEncoder.getRate(),
       wpimath.geometry.Rotation2d(1),
      )

   def GetPosition(self) -> wpimath.kinematics.SwerveModulePosition:
      print(f"Returning position of {self.DriveEncoder}")
     # # returns current position of the module
      return wpimath.kinematics.SwerveModulePosition(
         self.DriveEncoder.getDistancePerRotation()
         #self.DriveEncoder.getRate()
         ,
         wpimath.geometry.Rotation2d(1.0),
      )

   def SetState(self, desiredState: wpimath) -> None:
      encoderRotation = wpimath.geometry.Rotation2d(1.0)

      # Optimize the reference state to avoid spinning further than 90 degrees
      state = wpimath.kinematics.SwerveModuleState.optimize(
         desiredState, encoderRotation
      )
      print(f"Setting state to {self.DriveEncoder}")
      # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
      # direction of travel that can occur when modules change directions. This results in smoother
      # driving.
      state.speed *= (state.angle - encoderRotation
                      ).cos()

      # Calculate the drive output from the drive PID controller1.
      driveOutput = self.drivePIDController.calculate(
         self.DriveEncoder.getDistancePerRotation(),
         state.speed
      )

      driveFeedforward = self.driveFeedforward.calculate(state.speed)

      # Calculate the turning motor output from the turning PID controller1.
      turnOutput = self.turningPIDController.calculate(
         0.0,
         state.angle.radians()
      )

      turnFeedforward = self.turnFeedforward.calculate(
         self.turningPIDController.getSetpoint().velocity
      )
      self.DriveMotor.setVoltage(driveOutput + driveFeedforward)
      self.TurningMotor.setVoltage(turnOutput + turnFeedforward)
