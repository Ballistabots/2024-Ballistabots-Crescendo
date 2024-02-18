import robotcontainer as RobotContainer


class AutoAim():
   def __init__(self) -> None:
      self.Eyes = RobotContainer.Vison
      self.Arm = RobotContainer.Arm
      self.Shooter = RobotContainer.shooter
      self.pitch = 0

      self.CAMERA1_CAMERA_HEIGHT = 0  # in meters
      self.CAMERA1_PITCH = 0  # in rads

      self.TARGET_HEIGHT = 4.49  # height of the apriltag target in meters

   def Aim(self):
      """
      Aims the arm towards the optimal angle to have automatic shooting
      :return: None
      """
      results = self.Eyes.getResults()
      target = results.targets
      for i in target:
         self.pitch = i.getPitch()
      Rads = self.Eyes.PitchToRads(self.pitch)
      distance = self.Eyes.GetDistanceFromBestTag(self.CAMERA1_CAMERA_HEIGHT, self.TARGET_HEIGHT, self.CAMERA1_PITCH,
                                                  Rads)
      RequestedAngle = self.DistanceToAngle(distance)

      self.Arm.moveToPosition()

   def DistanceToAngle(self, distance: float):
      """
      used to find the optimal angle depending on the distance between the robot and speaker

      do this after tuning enc for distance
      test example: distance = 0 angle of shooter = 0, overshoot (or undershoot or completely missed)
      :param distance: Distance between the robot and speaker
      :return: Angle for the shooter to aim at
      """
      return 0

   def scale_number(self, unscaled: float, to_min: float, to_max: float, from_min: float, from_max: float):
      """

      :param unscaled: Number that you want to manipulate
      :param to_min: The minimum value to scale to
      :param to_max: The maximum value to scale to
      :param from_min: The minimum value that's getting scaled
      :param from_max: The maximum value that's getting scaled
      :return:  number
      """
      return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min
