
import wpimath.controller
import wpimath.geometry

import wpimath.trajectory

import robotcontainer


class PathTest():
   def __init__(self):
      self.robotContainer = robotcontainer.RobotContainer()
      self.DriveTrain = self.robotContainer.drivetrain

      pass

   trajectoryConfig = wpimath.trajectory.TrajectoryConfig(
      0.5,
      0.5
   )

   path = [
      wpimath.geometry.Translation2d(0.25,0)
   ]

   trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
      wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d(0)),
      path,
      wpimath.geometry.Pose2d(0.5, 0, wpimath.geometry.Rotation2d.fromDegrees(90)),
      trajectoryConfig
   )


   ramseteController = wpimath.controller.RamseteController()





