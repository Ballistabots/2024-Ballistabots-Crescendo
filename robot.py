import math

import wpilib
import wpilib.drive
import wpilib.drive
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds
import wpilib.shuffleboard
from robotcontainer import RobotContainer


class MyRobot(wpilib.TimedRobot):

   def __init__(self):
      super().__init__()
      board = wpilib.shuffleboard.Shuffleboard


   def disabledPeriodic(self):
      # self.drivetrain.gyro.zeroYaw()
      pass

   def robotInit(self):
      self.joystick = wpilib.Joystick(0)
      """
      This function is called upon program startup and
      should be used for any initialization code.
      """
      self.robotContainer = RobotContainer()
      self.drivetrain = self.robotContainer.drivetrain

   def autonomousInit(self):
      """This function is run once each time the robot enters autonomous mode."""
      pass

   def autonomousPeriodic(self):
      pass
      """This function is called periodically during autonomous."""
      """pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      goal_end_vel=0.0, # Goal end velocity in meters/sec
      rotation_delay_distance=0.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
      )"""

   def teleopInit(self):
      """This function is called once each time the robot enters teleoperated mode."""

      # self.drivetrain.gyro.set_yaw(0)

      """self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, 5.0))
      self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, 5.0))
      self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, 5.0))
      self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, 5.0))"""

   def teleopPeriodic(self):
      """This function is called periodically during teleoperated mode."""

      xspeed = self.joystick.getX()
      yspeed = self.joystick.getY()
      tspeed = self.joystick.getTwist()

      yaw = -self.drivetrain.gyro.getYaw()

      h = yaw % 360
      if h < 0:
         h += 360

      h2 = h / 360

      heading = h2 * (math.pi * 2)

      if abs(xspeed) < .15:
         xspeed = 0
      if abs(yspeed) < .15:
         yspeed = 0
      if abs(tspeed) < .15:
         tspeed = 0

      if xspeed == 0 and yspeed == 0 and tspeed == 0:  # xspeed == 0 and yspeed == 0 and tspeed == 0:
         self.drivetrain.frontLeftDrive.set(0)
         self.drivetrain.backRightDrive.set(0)
         self.drivetrain.backLeftDrive.set(0)
         self.drivetrain.frontRightDrive.set(0)

         self.drivetrain.backLeftRotation.set(0)
         self.drivetrain.backRightRotation.set(0)
         self.drivetrain.frontLeftRotation.set(0)
         self.drivetrain.frontRightRotation.set(0)
      else:
         # field centric
         # speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-yspeed * 0.5, xspeed * 0.5, tspeed * 0.5, Rotation2d(self.drivetrain.gyro.getRoll()))
         # robot centric
         speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-yspeed * self.slow, xspeed * self.slow, tspeed,
                                                        Rotation2d(heading))
         print(heading)
         self.drivetrain.driveFromChassisSpeeds(speeds)

   def robotPeriodic(self):
      self.slow = -self.joystick.getThrottle()


if __name__ == "__main__":
   wpilib.run(MyRobot)
