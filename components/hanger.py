import wpilib


class Hanger():

   def __init__(self):

      #self.compressor = wpilib.Compressor(21,wpilib.PneumaticsModuleType.REVPH)

      # DoubleSolenoid corresponds to a double solenoid.
      # In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
      self.doubleSolenoid = wpilib.DoubleSolenoid(21,
         moduleType=wpilib.PneumaticsModuleType.REVPH,
         forwardChannel=8,
         reverseChannel=9,
      )

   def DeployHang(self, buttonForward:bool,buttonBackwards:bool) -> None:
      """


      :return: Nothing
      """
      if buttonForward:
         self.doubleSolenoid.set(self.doubleSolenoid.Value.kForward)
      elif buttonBackwards:
         self.doubleSolenoid.set(self.doubleSolenoid.Value.kReverse)
