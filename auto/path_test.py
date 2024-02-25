import math

import wpimath.controller
import wpimath.geometry

import wpimath.trajectory

import robotcontainer


class PathTest():
   def __init__(self):
      self.robotContainer = robotcontainer.RobotContainer()
      self.driveTrain = self.robotContainer.drivetrain

      pass

   trajectoryConfig = wpimath.trajectory.TrajectoryConfig(
      3,
      1
   )

   path = [
      wpimath.geometry.Translation2d(1, 0),
      wpimath.geometry.Translation2d(1, -1)

   ]

   trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
      wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d(0)),
      path,
      wpimath.geometry.Pose2d(2, -1, wpimath.geometry.Rotation2d.fromDegrees(180)),
      trajectoryConfig
   )
   maxAngularSpeedRadiansPerSecond = 2 * 2 * math.pi
   kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4

   turnConst = wpimath.trajectory.TrapezoidProfile.Constraints(
      wpimath.trajectory.TrapezoidProfile.Constraints(
         maxAngularSpeedRadiansPerSecond,
         kMaxAngularAccelerationRadiansPerSecondSquared
      )
   )

   # pid aojsknfKSJEFNkjwfnkjEWFM V

   xKp = 1
   yKp = 1
   zKp = 1

   xController = wpimath.controller.PIDController(xKp, 0, 0)
   yController = wpimath.controller.PIDController(yKp, 0, 0)

   turnPid = wpimath.controller.ProfiledPIDController(
      zKp, 0, 0,
      turnConst
   )

   turnPid.enableContinuousInput(-math.pi, math.pi)

   ramseteController = wpimath.controller.RamseteController()




