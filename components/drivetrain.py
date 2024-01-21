import math
import wpimath.geometry
import wpimath.kinematics
import components.swervemodule as swervemodule

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
   def __init__(self) -> None:
      # sets locations of the modules
      self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
      self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
      self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
      self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

      # creates the motor objects
      self.frontLeft = swervemodule.SwerveModule(0, 0, 0, 0, 0, 0)
      self.frontRight = swervemodule.SwerveModule(0, 0, 0, 0, 0, 0)
      self.backLeft = swervemodule.SwerveModule(0, 0, 0, 0, 0, 0)
      self.backRight = swervemodule.SwerveModule(0, 0, 0, 0, 0, 0)

      # self.gyro = wpilib.AnalogGyro(0)

      self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
         # makes kinematics happy
         self.frontLeftLocation,
         self.frontRightLocation,
         self.backLeftLocation,
         self.backRightLocation,
      )

      """
      #odometry without odometry pods :(
      self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
         #used for path planning and auto
         self.kinematics,
         self.gyro.getRotation2d(),
         (
            self.frontLeft.GetPosition(),
            self.frontRight.GetPosition(),
            self.backLeft.GetPosition(),
            self.backRight.GetPosition(),
         ),
      )
      """

      # self.gyro.reset()
      def drive(
              self,
              xSpeed: float,
              ySpeed: float,
              rot: float,
              fieldRelative: bool,
              periodSeconds: float,
      ) -> None:
         """
         Method to drive the robot using joystick info.
         :param xSpeed: Speed of the robot in the x direction (forward).
         :param ySpeed: Speed of the robot in the y direction (sideways).
         :param rot: Angular rate of the robot.
         :param fieldRelative: Whether the provided x and y speeds are relative to the field.
         :param periodSeconds: Time
         """
         swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
               wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed, ySpeed, rot, self.gyro.getRotation2d()
               )
               if fieldRelative
               else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
               periodSeconds,
            )
         )
         wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
         )
         self.frontLeft.setDesiredState(swerveModuleStates[0])
         self.frontRight.setDesiredState(swerveModuleStates[1])
         self.backLeft.setDesiredState(swerveModuleStates[2])
         self.backRight.setDesiredState(swerveModuleStates[3])


"""
   def updateOdometry(self) -> None:
      #Updates the field relative position of the robot
      self.odometry.update(
         self.gyro.getRotation2d(),
         (
            self.frontLeft.GetPosition(),
            self.frontRight.GetPosition(),
            self.backLeft.GetPosition(),
            self.backRight.GetPosition(),
         ),
      )

"""
