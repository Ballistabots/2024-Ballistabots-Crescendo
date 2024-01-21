import rev as Rev
import wpilib
import wpilib.drive
from phoenix6.hardware.core.core_talon_fx import CoreTalonFX


class MyRobot(wpilib.TimedRobot):
   # nothinng
   # print("nothing please leave me alone this is nothing im not crazy, crazy? i was crazy once, they locked me in a room, a rubber room. a rubber room with rats. RATS? rats make me crazy, crazy?")
   def robotPeriodic(self):
      pass


   def robotInit(self) -> None:
      self.frontLeft = Rev.CANSparkMax(7, Rev.CANSparkMax.MotorType.kBrushless)
      self.frontRight = Rev.CANSparkMax(5, Rev.CANSparkMax.MotorType.kBrushless)
      self.backLeft = Rev.CANSparkMax(1, Rev.CANSparkMax.MotorType.kBrushless)
      self.backRight = Rev.CANSparkMax(6, Rev.CANSparkMax.MotorType.kBrushless)


      self.outake = CoreTalonFX(1,'rio')
      self.intake = Rev.CANSparkMax(2, Rev.CANSparkMax.MotorType.kBrushless)
      self.belt = Rev.CANSparkMax(3,Rev.CANSparkMax.MotorType.kBrushless)

      self.slow = 0.5

      self.frontRight.setInverted(True)

      self.backRight.setInverted(True)



      self.robotDrive = wpilib.drive.MecanumDrive(self.frontLeft, self.backLeft, self.frontRight,self.backRight)
      self.driver = wpilib.Joystick(0)

   def teleopPeriodic(self):
      self.robotDrive.driveCartesian(
         -self.driver.getY() * self.slow,
         -self.driver.getX() * self.slow,
         -self.driver.getZ() * self.slow,
      )
      if self.driver.getTrigger():
         self.intake.setVoltage(-30)
         self.belt.setVoltage(30)
      else:
         self.intake.setVoltage(0)
         self.belt.setVoltage(0)

      if self.driver.getThrottle() > .30:
          #figure this out please !!!!! self.outake.
          print("shot")

if __name__ == "__main__":
    wpilib.run(MyRobot)