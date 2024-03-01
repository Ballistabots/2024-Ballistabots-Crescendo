
import wpimath.controller
import wpimath.geometry

import wpimath.trajectory

import robotcontainer


class PathTest():
   def __init__(self):
      self.robotContainer = robotcontainer.RobotContainer()
      self.DriveTrain = self.robotContainer.drivetrain
      # self.trajectoryConfig = wpimath.trajectory.TrajectoryConfig

      pass

   trajectoryConfig = wpimath.trajectory.TrajectoryConfig(
      1,
      1
   )

   path = [
      wpimath.geometry.Translation2d(0.3, 0)

   ]

   trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
      wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d(0)),
      path,
      wpimath.geometry.Pose2d(0.6, 0, wpimath.geometry.Rotation2d.fromDegrees(0)),
      trajectoryConfig
   )
   ramseteController = wpimath.controller.RamseteController()


