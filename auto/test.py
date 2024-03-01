import math

import commands2
import wpimath.trajectory
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
from wpimath._controls._controls.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath._controls._controls.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import robotcontainer

class Test():
   def __init__(self):
      self.drivetrain = robotcontainer.DriveTrain
      self.trajectoryConfig = TrajectoryConfig(
        3,1
      )
      self.trajectoryConfig.setKinematics(self.drivetrain.getKinematics())

      self.trajectory = TrajectoryGenerator.generateTrajectory(
         # ? initial location and rotation
         Pose2d(0, 0, Rotation2d(0)),
         [
            # ? points we want to hit
            Translation2d(1, 0),
            Translation2d(1, -1),
            Translation2d(0, -1),
         ],
         # ? final location and rotation
         Pose2d(0, 0, Rotation2d(math.pi)),
         self.trajectoryConfig
      )
      xKp = 0.001
      yKp = 0.001

      turnKp = 0.001

      turnConstants = wpimath.trajectory.TrapezoidProfile.Constraints(
         3,1

      )

      self.xController = PIDController(xKp, 0, 0)
      self.yController = PIDController(yKp, 0, 0)

      self.thetaController = ProfiledPIDControllerRadians(
         turnKp, 0, 0, turnConstants)
      self.thetaController.enableContinuousInput(-math.pi, math.pi)

      pathFollowerConfig = HolonomicPathFollowerConfig(
         PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
         PIDConstants(5.0, 0.0, 1.0),  # Rotation PID constants , # PIDConstants(9.0, 0.0, 0.5),
         3,  # Max module speed, in m/s
         1/3,
         # Drive base radius in meters. Distance from robot center to furthest module.
         ReplanningConfig()  # Default path replanning config. See the API for the options here
      )

      driveController = HolonomicDriveController(
         self.xController,
         self.yController,
         self.thetaController
      )

      self.swerveControllerCommand1 = commands2.SwerveControllerCommand(
         self.trajectory,
         self.drivetrain.getPose,
         self.drivetrain.getKinematics(),
         driveController,
         self.drivetrain.setSwivel
      )
      self.sCurve = commands2.SequentialCommandGroup(
         self.swerveControllerCommand1
      )

   def getCommand(self):
      return self.swerveControllerCommand1

