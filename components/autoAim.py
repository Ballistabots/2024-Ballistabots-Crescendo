import robotcontainer as RobotContainer


class AutoAim():
   def __init__(self) -> None:
      self.Eyes = RobotContainer.Vison
      self.redSideTag = 10  # change both of these
      self.blueSideTag = 11
      pass

   def Enabled(self, Enable: bool) -> None:
      """Function for enable autoAim via controller input or any boolean based input

      """

      pass

   def getDistanceFromTag(self,tagId:int) -> float:
      """Returns the estimated distance from the april tag"""
      tagSize = self.Eyes.GetTagSize(tagId)
      scaledDistance = self.scale_number(tagSize,3,10,0.001,1.0)
      return scaledDistance

   def getEncFromDistance(self,distance:float) -> float:
      scaledEnc = self.scale_number(distance,0.001,2.4,3,10)#change the to_min and t0_max while tuning
      return scaledEnc

   def Aim(self,enc:float):
      pidController = None




   def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
      return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min
