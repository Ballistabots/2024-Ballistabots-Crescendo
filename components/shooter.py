
import navx
import phoenix6
import rev
import wpilib


class shooter():

   def __init__(self):
      self.Outtake1 = phoenix6.hardware.TalonFX(15, "rio")  # falcon500
      self.Outtake2 = phoenix6.hardware.TalonFX(16, "rio")

      self.control = phoenix6.controls.DutyCycleOut(0)

      self.Intake = rev.CANSparkMax(17, rev.CANSparkMax.MotorType.kBrushless)

      self.ShooterGyro = navx.AHRS.create_i2c(wpilib.I2C.Port(0))

   def Intake(self, button:bool, toggle:bool = False) -> None:
      if toggle:
         self.Intake.set(1.0)
      else:
         if button:
            self.Intake.set(1.0)
         else:
            self.Intake.set(0.0)

   def Outtake(self, velocity: float) -> None:
      self.Outtake1.set_control(self.control.with_output(velocity))
      self.Outtake2.set_control(phoenix6.controls.Follower(15,True))


   def OuttakeButIntake(self, velocity: float) -> None:
      self.Outtake1.set_control(self.control.with_output(-velocity))
      self.Outtake2.set_control(phoenix6.controls.Follower(15, True))


   def ShooterGyroOdo(self) -> tuple:
      """

      used to find the position of the shooter
      :return: tuple of yaw, pitch and angle in that order
      """
      return (self.ShooterGyro.getYaw(), self.ShooterGyro.getPitch(), self.ShooterGyro.getAngle())
