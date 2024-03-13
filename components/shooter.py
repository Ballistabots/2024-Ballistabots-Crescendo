import phoenix6
import rev
import wpilib




class shooter():

   def __init__(self):
      self.Outtake1 = phoenix6.hardware.TalonFX(15, "rio")  # falcon500
      self.Outtake2 = phoenix6.hardware.TalonFX(16, "rio")

      self.control = phoenix6.controls.DutyCycleOut(0)

      self.stop = True

      self.Intake = rev.CANSparkMax(14, rev.CANSparkMax.MotorType.kBrushless)

      self.timer = wpilib.Timer()

      self.timer.start()






      # self.ShooterGyro = navx.AHRS.create_i2c(wpilib.I2C.Port(0))
      # self.ShooterGyro.enableLogging(True)


   def fancy_intake(self,intake:bool,reverse_intake:bool, stop:bool) -> None:

      if self.Intake.getOutputCurrent() > 0:
         print(self.Intake.getOutputCurrent())
      if self.Intake.getOutputCurrent() > 18.1 :
         print("Stoping")
         self.stop = True
      if intake:
         self.timer.reset()
         self.timer.start()
         self.stop = False


      elif reverse_intake:
         self.Intake.set(-0.1)
      elif stop:
         self.Intake.set(0)

      if self.stop == False:

         self.Intake.set(0.3)
         if stop:
            self.stop = True
            self.Intake.set(0)
      elif self.stop:
         self.Intake.set(0)




   def getTime(self):
      return self.timer.get()

   def SetIntakePower(self, power: float) -> None:
      self.Intake.set(power)

   def Outtake(self, velocity: float) -> None:
      self.Outtake1.set_control(self.control.with_output(velocity))
      self.Outtake2.set_control(phoenix6.controls.Follower(15, False))
      #self.Intake.set(-velocity)

   def OuttakeButIntake(self, velocity: float) -> None:
      """
      intakes using the outtake motors for source
      :param velocity: velocity given to the motors
      :return: Nothing lol
      """
      self.Outtake1.set_control(self.control.with_output(-velocity))
      self.Outtake2.set_control(phoenix6.controls.Follower(15, True))
      #self.Intake.set(velocity)

   def ShooterGyroOdo(self) -> tuple:
      """
      used to find the position of the shooter
      :return: tuple of yaw, pitch and angle in that order
      """
      return (3, 3, 3)
      # (self.ShooterGyro.getYaw(), self.ShooterGyro.getPitch(), self.ShooterGyro.getAngle())
