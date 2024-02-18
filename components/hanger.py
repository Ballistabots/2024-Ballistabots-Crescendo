<<<<<<< HEAD
import wpilib



class Hanger():

   def __init__(self):
      self.joystick = wpilib.Joystick(0)

      self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.REVPH)

      self.kSolenoidButton = 1
      self.kDoubleSolenoidForwardButton = 2
      self.kDoubleSolenoidReverseButton = 3
      self.kCompressorButton = 4

      # DoubleSolenoid corresponds to a double solenoid.
      # In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
      self.doubleSolenoid = wpilib.DoubleSolenoid(
         moduleType=wpilib.PneumaticsModuleType.REVPH,
         forwardChannel=1,
         reverseChannel=2,
      )


      """
      # The output of GetRawButton is true/false depending on whether
         # the button is pressed; Set takes a boolean for whether
         # to retract the solenoid (false) or extend it (true).
         self.solenoid.set(self.joystick.getRawButton(6))

         # GetRawButtonPressed will only return true once per press.
         # If a button is pressed, set the solenoid to the respective channel.

         if self.joystick.getRawButtonPressed(11):
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
         elif self.joystick.getRawButtonPressed(12):
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

         # On button press, toggle the compressor.
         if self.joystick.getRawButtonPressed(self.kCompressorButton):
            # Check whether the compressor is currently enabled.
            isCompressorEnabled = self.compressor.isEnabled()
            if isCompressorEnabled:
               # Disable closed-loop mode on the compressor
               self.compressor.disable()
            else:
               # Change the if states to select the closed-loop you want to use:

               if False:
                  # Enable the closed-lopp mode base on the digital pressure switch connect to the PCM/PH.
                  # The switch is open when the pressure is over ~120 PSI.
                  self.compressor.enableDigital()

               if True:
                  # Enable closed-loop mode based on the analog pressure sensor connected to the PH.
                  # The compressor will run while the pressure reported by the sensor is in the
                  # specified range ([70 PSI, 120 PSI] in this example).
                  # Analog mode exists only on the PH! On the PCM, this enables digital control.
                  self.compressor.enableAnalog(70, 120)

               if False:
                  # Enable closed-loop mode based on both the digital pressure switch AND the analog
                  # pressure sensor connected to the PH.
                  # The compressor will run while the pressure reported by the analog sensor is in the
                  # specified range ([70 PSI, 120 PSI] in this example) AND the digital switch reports
                  # that the system is not full.
                  # Hybrid mode exists only on the PH! On the PCM, this enables digital control.
                  self.compressor.enableHybrid(70, 120)
      
      
      """

=======
import wpilib



class Hanger():

   def __init__(self):
      self.joystick = wpilib.Joystick(0)

      self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.REVPH)

      self.kSolenoidButton = 1
      self.kDoubleSolenoidForwardButton = 2
      self.kDoubleSolenoidReverseButton = 3
      self.kCompressorButton = 4

      # DoubleSolenoid corresponds to a double solenoid.
      # In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
      self.doubleSolenoid = wpilib.DoubleSolenoid(
         moduleType=wpilib.PneumaticsModuleType.REVPH,
         forwardChannel=1,
         reverseChannel=2,
      )


      """
      # The output of GetRawButton is true/false depending on whether
         # the button is pressed; Set takes a boolean for whether
         # to retract the solenoid (false) or extend it (true).
         self.solenoid.set(self.joystick.getRawButton(6))

         # GetRawButtonPressed will only return true once per press.
         # If a button is pressed, set the solenoid to the respective channel.

         if self.joystick.getRawButtonPressed(11):
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
         elif self.joystick.getRawButtonPressed(12):
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

         # On button press, toggle the compressor.
         if self.joystick.getRawButtonPressed(self.kCompressorButton):
            # Check whether the compressor is currently enabled.
            isCompressorEnabled = self.compressor.isEnabled()
            if isCompressorEnabled:
               # Disable closed-loop mode on the compressor
               self.compressor.disable()
            else:
               # Change the if states to select the closed-loop you want to use:

               if False:
                  # Enable the closed-lopp mode base on the digital pressure switch connect to the PCM/PH.
                  # The switch is open when the pressure is over ~120 PSI.
                  self.compressor.enableDigital()

               if True:
                  # Enable closed-loop mode based on the analog pressure sensor connected to the PH.
                  # The compressor will run while the pressure reported by the sensor is in the
                  # specified range ([70 PSI, 120 PSI] in this example).
                  # Analog mode exists only on the PH! On the PCM, this enables digital control.
                  self.compressor.enableAnalog(70, 120)

               if False:
                  # Enable closed-loop mode based on both the digital pressure switch AND the analog
                  # pressure sensor connected to the PH.
                  # The compressor will run while the pressure reported by the analog sensor is in the
                  # specified range ([70 PSI, 120 PSI] in this example) AND the digital switch reports
                  # that the system is not full.
                  # Hybrid mode exists only on the PH! On the PCM, this enables digital control.
                  self.compressor.enableHybrid(70, 120)
      
      
      """

>>>>>>> b620496b295763c01a348429f66fce13c36cf3bf
