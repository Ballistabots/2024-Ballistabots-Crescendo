import phoenix6


class Arm():

   def __init__(self):
      self.Arm1 = phoenix6.hardware.TalonFX(17, "rio")
      self.Arm2 = phoenix6.hardware.TalonFX(18, "rio")



      self.Arm1follower = phoenix6.controls.Follower(17, True)

      self.position_request = phoenix6.controls.PositionVoltage(0, 0,feed_forward=-0.22) #0.0075

      # cfg = phoenix6.configs.TalonFXConfiguration()

      testConfig = phoenix6.configs.TalonFXConfiguration()
      testConfig.slot0.with_k_p(0.006)
      testConfig.slot0.with_k_i(0)
      testConfig.slot0.with_k_d(0)


      self.Arm1.configurator.apply(testConfig)



      # self.robotContainer = robotcontainer.RobotContainer()

      # self.Shooter = robotcontainer.shooter

   def ArmStartingPos(self):
      """
      sets the arm to its starting pose
      :return: Nothing
      """
      self.Arm1.set_position(0)
      self.Arm2.set_control(self.Arm1follower)

   def moveToEncoderPos(self, enc_pos):
      self.Arm1.set_position(enc_pos)
      self.Arm2.set_control(self.Arm1follower)

   def moveToPosition(self, newPos: float):
      """
      Moves the arm to the specified position
      :param newPos: The position to move the arm to
      :return: Nothing
      """
      self.Arm1.set_control(self.position_request.with_position(newPos).with_velocity(1))
      self.Arm2.set_control(self.Arm1follower)


   def getArmPosition(self) -> float:
      """
      used to get the position of the arm
      :return: The Position of the Arm
      """
      return self.Arm1.get_position().value_as_double

   def EncToAngle(self, enc) -> float:
      """
      used to convert an encoder value to an angle value
      :param enc: encoder value used as input
      :return: translated angle value
      """
      conversion = 2048 / 360
      return enc * conversion

   def AngleToEnc(self, angle) -> float:
      """
      used to convert an angle value to an encoder value
      :param angle: angle value used as input
      :return:Translated encoder value
      """
      conversion = 360 / 2048
      return angle * conversion

   def Spin(self, vel):
      self.Arm1.set_control(phoenix6.controls.DutyCycleOut(vel))
      self.Arm2.set_control(self.Arm1follower)

   def moveToAngleShooter(self, RequestedAngle: float):


      self.Arm1.set_control(self.position_request.with_position(RequestedAngle).with_velocity(1))
      self.Arm2.set_control(self.Arm1follower)

