import math

import wpilib
import wpilib.drive
import wpilib.drive
import wpilib.shuffleboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

from robotcontainer import RobotContainer


class State():
   def __init__(self, state: str):
      self.state = state
      pass

   def changeState(self, state: str):
      self.state = state

   def getState(self):
      return self.state


class MyRobot(wpilib.TimedRobot):

   def __init__(self):
      super().__init__()
      self.driver1 = wpilib.Joystick(0)
      self.driver2 = wpilib.Joystick(1)
      # self.joystickPS5 = wpilib.PS5Controller(0)
      board = wpilib.shuffleboard.Shuffleboard
      self.state = State('Disabled')

   def disabledPeriodic(self):
      # self.drivetrain.gyro.zeroYaw()
      pass

   def robotInit(self):
      """
      This function is called upon program startup and
      should be used for any initialization code.
      """
      # self.camera1 = photonlibpy.photonCamera.PhotonCamera("Camera1")

      self.robotContainer = RobotContainer()
      self.drivetrain = self.robotContainer.drivetrain
      self.shooter = self.robotContainer.shooter

      self.BleftRotation = self.robotContainer.drivetrain.backLeftRotation
      self.FleftRotation = self.robotContainer.drivetrain.frontLeftRotation
      self.BrightRotation = self.robotContainer.drivetrain.backRightRotation
      self.FrightRotation = self.robotContainer.drivetrain.frontRightRotation

      self.BleftPID = self.robotContainer.drivetrain.BleftPID
      self.FleftPID = self.robotContainer.drivetrain.FleftPID
      self.BrightPID = self.robotContainer.drivetrain.BrightPID
      self.FrightPID = self.robotContainer.drivetrain.FrightPID

      self.BleftEnc = self.robotContainer.drivetrain.BleftEnc
      self.FleftEnc = self.robotContainer.drivetrain.FleftEnc
      self.BrightEnc = self.robotContainer.drivetrain.BrightEnc
      self.FrightEnc = self.robotContainer.drivetrain.FrightEnc

   def autonomousInit(self):
      """This function is run once each time the robot enters autonomous mode."""
      # self.drivetrain.gyro.zeroYaw()
      pass

   def autonomousPeriodic(self):
      pass

   def teleopInit(self):
      """This function is called once each time the robot enters teleoperated mode."""

      # self.drivetrain.gyro.zeroYaw()  # remove this after testing
      self.slow = 0.3  # slows down the robots max speed

   def teleopPeriodic(self):
      """This function is called periodically during teleoperated mode."""
      self.robotContainer.StateHandler.match_state(self.state.getState())

      xspeed = self.driver1.getX()
      yspeed = self.driver1.getY()
      tspeed = self.driver1.getZ()

      yaw = -self.drivetrain.gyro.getYaw()

      if self.driver1.getRawButtonPressed(2):
         self.drivetrain.gyro.zeroYaw()

      h = yaw % 360  # formula to transform the yaw given by the gyro into a heading
      if h < 0:
         h += 360

      h2 = h / 360

      heading = h2 * (math.pi * 2)

      if abs(xspeed) < .15:  # applies  a deadzone to the joystick
         xspeed = 0
      if abs(yspeed) < .15:
         yspeed = 0
      if abs(tspeed) < .15:
         tspeed = 0

      if xspeed == 0 and yspeed == 0 and tspeed == 0:  # if no speed is given to the motors there will be no power in any of the motors
         self.drivetrain.frontLeftDrive.set(0)
         self.drivetrain.backRightDrive.set(0)
         self.drivetrain.backLeftDrive.set(0)
         self.drivetrain.frontRightDrive.set(0)

         self.drivetrain.backLeftRotation.set(0)
         self.drivetrain.backRightRotation.set(0)
         self.drivetrain.frontLeftRotation.set(0)
         self.drivetrain.frontRightRotation.set(0)

         # speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-yspeed * 0.5, xspeed * 0.5, tspeed * 0.5, Rotation2d(0.0))

         speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-yspeed * self.slow, xspeed * self.slow, -tspeed, Rotation2d(
            heading))  # calculates power given to the motors depending on the user inputs
         self.drivetrain.driveFromChassisSpeeds(speeds)
         # print(heading)



   def testInit(self):
      #on test init
      pass

   def testPeriodic(self):
      #loops while the test is enabled
      self.shooter.Outtake(self.driver2.getY())

   def robotPeriodic(self):
      #while the robot is on
      pass


if __name__ == "__main__":
   #is able to run the robot as an executable rather than a regular file
   wpilib.run(MyRobot)
