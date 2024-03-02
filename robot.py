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
      # self.path_test = self.robotContainer.path_test

      self.arm = self.robotContainer.arm
      #self.shooter = self.robotContainer.shooter

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

      pass

   def autonomousPeriodic(self):
      pass

   def teleopInit(self):
      """This function is called once each time the robot enters teleoperated mode."""

      # self.drivetrain.gyro.zeroYaw()  # remove this after testing
      self.slow = 0.7  # slows down the robots max speed



   def teleopPeriodic(self):
      """This function is called periodically during teleoperated mode."""
      xspeed = self.driver1.getX()
      yspeed = self.driver1.getY()

      if self.driver1.getRawButtonPressed(2):
         self.drivetrain.gyro.zeroYaw()

      if self.driver1.getTrigger():
         tspeed = self.driver1.getZ()
      else:
         tspeed = 0

      if abs(xspeed) < .2:  # applies  a deadzone to the joystick
         xspeed = 0
      if abs(yspeed) < .2:
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

      speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow, -tspeed,
                                                     Rotation2d(
                                                        self.heading))  # calculates power given to the motors depending on the user inputs
      self.drivetrain.driveFromChassisSpeeds(speeds)

      self.arm_value = self.driver2.getY() * 500
      self.arm.moveToPosition(self.arm_value)

      #shooterPower = self.driver2.getThrottle()

      #if abs(shooterPower) < 0.2:
       #  shooterPower = 0

      #self.shooter.Outtake(shooterPower)

      # self.shooter.Intake(button=self.driver2.getRawButtonPressed(2))

   def testInit(self):
      # on test init
      pass

   def testPeriodic(self):
      pass

   def robotPeriodic(self):
      # while the robot is on
      # print(self.drivetrain.odometry.getPose())
      self.yaw = -self.drivetrain.gyro.getAngle()

      h = self.yaw % 360  # formula to transform the yaw given by the gyro into a heading
      if h < 0:
         h += 360

      self.h2 = h / 360

      self.heading = self.h2 * (math.pi * 2)


if __name__ == "__main__":
   # is able to run the robot as an executable rather than a regular file
   wpilib.run(MyRobot)
