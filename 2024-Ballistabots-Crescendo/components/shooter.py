import navx
import phoenix6
import rev
import wpilib


class shooter():

   def __init__(self):
      self.Outtake1 = phoenix6.hardware.TalonFX(15, "rio")  # falcon500
      self.Outtake2 = phoenix6.hardware.TalonFX(16, "rio")
      self.Intake = rev.CANSparkMax(17, rev.CANSparkMax.MotorType.kBrushless)

      self.ShooterGyro = navx.AHRS.create_i2c(wpilib.I2C.Port(0))

   def Intake(self):
      self.Intake.set(1.0)

   def Outtake(self, velocity: float, acceleration: float) -> None:
      pass

   def OuttakeButIntake(self, velocity: float, acceleration: float) -> None:
      # self.Outtake1.set_position()
      pass

   def ShooterGyroOdo(self) -> tuple:
      """

      used to find the position of the shooter
      :return: tuple of yaw, pitch and angle in that order
      """
      return (self.ShooterGyro.getYaw(), self.ShooterGyro.getPitch(), self.ShooterGyro.getAngle())
