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
      self.joystick = wpilib.Joystick(0)
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
      # self.camera = photonlibpy.photonCamera.PhotonCamera("Camera1")

      self.robotContainer = RobotContainer()
      self.drivetrain = self.robotContainer.drivetrain

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

      # self.BleftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, 5.0))
      # self.FleftRotation.set(self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, 5.0))
      # self.BrightRotation.set(self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, 5.0))
      # self.FrightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, 5.0))

      # self.drivetrain.gyro.zeroYaw()  # remove this after testing
      self.slow = 0.3

   def teleopPeriodic(self):
      """This function is called periodically during teleoperated mode."""
      self.robotContainer.StateHandler.match_state(self.state.getState())

      # result = self.camera.getLatestResult()

      # id = result.getTargets()

      # for target in id:

      # wpilib.SmartDashboard.putNumber("ID",target.getFiducialId())
      # wpilib.SmartDashboard.putNumber("YAW",target.getYaw())
      # wpilib.SmartDashboard.putNumber("AREA",target.getArea())

      # wpilib.SmartDashboard.putString("Area On the Field", self.robotContainer.vision.aprilTags[target.getFiducialId()])

      xspeed = self.joystick.getX()
      yspeed = self.joystick.getY()
      tspeed = self.joystick.getZ()

      # yaw = -self.drivetrain.gyro.getRawGyroY()
      yaw = -self.drivetrain.gyro.getYaw()

      if self.joystick.getRawButtonPressed(2):
         self.drivetrain.gyro.zeroYaw()

      # if self.joystick.getRawButtonPressed(1):

      #  self.state.changeState("Aligning")
      # self.drivetrain.align(self.robotContainer.vision.getTagOdometry(10 or 11))  # aligns the robot with april tag
      # getting shooting angle goes here
      # self.robotContainer.auto_aim.Aim()  # aims the arm and gets ready to shoot

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
         if self.state.getState() == "Shooting":
            self.state.changeState("Shooting_Driving")
            pass
         elif self.state.getState() == "Aiming":
            self.state.changeState("Aiming_Driving")

         else:
            self.state.changeState("Driving")

         # speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-yspeed * 0.5, xspeed * 0.5, tspeed * 0.5, Rotation2d(0.0))

         speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-yspeed * self.slow, xspeed * self.slow, -tspeed,
                                                        Rotation2d(heading))
         self.drivetrain.driveFromChassisSpeeds(speeds)
         # print(heading)



   def robotPeriodic(self):
      pass


if __name__ == "__main__":
   wpilib.run(MyRobot)
