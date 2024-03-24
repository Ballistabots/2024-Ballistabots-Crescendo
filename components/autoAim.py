import robotcontainer


class AutoAim():
   def __init__(self) -> None:

      self.Eyes = robotcontainer.Vision()
      self.pitch = 0

   def Aim(self):
      """
      Aims the arm towards the optimal angle to have automatic shooting
      :return: speed for motors to spin relative to the distance
      """
      results = self.Eyes.getResults()
      if results.hasTargets():
         target = results.targets
         for i in target:

            if i.getFiducialId() == 3 or i.getFiducialId() == 7:
               self.id = i.getFiducialId()
               self.pitch = i.getPitch()
            else:
               self.pitch = 0

         if self.pitch != 0:
            distance = self.Eyes.GetDistanceFromSpeaker(self.pitch)
            # distance to arm encoder stuff


            return (self.scale_number(distance, 0.6, 1, 1, 14/3),self.scale_number(distance,12.4,36,1,14/3))#where the arm needs to be aimed
         else:
            print("No tag")
            pass
      else:
         return (0.1,0)
         pass

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
