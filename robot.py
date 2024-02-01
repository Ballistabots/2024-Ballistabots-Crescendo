import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from components import drivetrain


class MyRobot(wpilib.TimedRobot):
   def robotInit(self) -> None:
      """Robot initialization function"""
      self.controller1 = wpilib.XboxController(0)
      #self.controller1 = wpilib.Joystick(0)
      #self.cotroller1 = wpilib.PS5Controller(0)
      self.swerve = drivetrain.Drivetrain()

      # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
      self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
      self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
      self.rotLimiter = wpimath.filter.SlewRateLimiter(3)


def autonomousPeriodic(self) -> None:
   self.driveWithJoystick(False)
   #self.swerve.updateOdometry()
   print(self.swerve.DriveTo(10,10,1))



def teleopPeriodic(self) -> None:
   self.driveWithJoystick(False)


def driveWithJoystick(self, fieldRelative: bool) -> None:
   # Get the x speed. We are inverting this because Xbox controllers return
   # negative values when we push forward.
   xSpeed = (
           -self.xspeedLimiter.calculate(
              wpimath.applyDeadband(self.controller1.getLeftY(), 0.02)#dont ask me what this is for cause i have no idea
           )
           * drivetrain.kMaxSpeed
   )

   # Get the y speed or sideways/strafe speed we are inverting this because
   # we want a positive value when we pull to the left. Xbox controllers
   # return positive values when you pull to the right by default.
   ySpeed = (
           -self.yspeedLimiter.calculate(
              wpimath.applyDeadband(self.controller1.getLeftX(), 0.02)#dont ask me what this is for cause i have no idea
           )
           * drivetrain.kMaxSpeed
   )

   # Get the rate of angular rotation. We are inverting this because we want a
   # positive value when we pull to the left (remember, CCW is positive in
   # mathematics). Xbox controllers return positive values when you pull to
   # the right by default.
   rot = (
           -self.rotLimiter.calculate(
              wpimath.applyDeadband(self.controller1.getRightX(), 0.02)#dont ask me what this is for cause i have no idea
           )
           * drivetrain.kMaxSpeed
   )

   self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

