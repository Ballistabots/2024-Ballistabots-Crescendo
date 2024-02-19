import wpilib


class Hanger():

   def __init__(self):
      self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

      # DoubleSolenoid corresponds to a double solenoid.
      # In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
      self.doubleSolenoid = wpilib.DoubleSolenoid(
         moduleType=wpilib.PneumaticsModuleType.REVPH,
         forwardChannel=0,
         reverseChannel=1,
      )

   def DeployHang(self, on: bool) -> None:
      """

      :param on: Using a button input to determine whether or not the hanger goes up or down
      :return: Nothing
      """
      if on:
         self.doubleSolenoid.set(self.doubleSolenoid.Value.kForward)
      else:
         self.doubleSolenoid.set(self.doubleSolenoid.Value.kReverse)
