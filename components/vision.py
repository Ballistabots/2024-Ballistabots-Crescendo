import math

import photonlibpy.photonCamera


class Vison:
   def __init__(self):
      self.camera1 = photonlibpy.photonCamera.PhotonCamera(
         "Camera1")  # initializes a camera using the photonlib library

      self.CAMERA1_CAMERA_HEIGHT = 0  # in meters. height the camera is at
      self.CAMERA1_PITCH = 0  # in rads. where the camera is pointing

      self.TARGET_HEIGHT = 4.49  # height of the apriltag target in meters

      self.aprilTags = (0,  # large array full of apriltag names for quick access
                        "Blue Source Right",
                        "Blue Source Left",
                        "Red Speaker Right",
                        "Red Speaker Left",
                        "Red Amp",
                        "Blue Amp",
                        "Blue Speaker Right",
                        "Blue Speaker Left",
                        "Red Source Right",
                        "Red Source Left",
                        "Red Stage Left",
                        "Red Stage Middle",
                        "Red Stage Right",
                        "Blue Stage Left",
                        "Blue Stage Middle",
                        "Blue Stage Right"
                        )

   def getResults(self) -> photonlibpy.photonCamera.PhotonPipelineResult:
      """

      :return: results given by the camera as a object to be used for data parsing
      """
      return self.camera1.getLatestResult()

   def PitchToRads(self, pitch: float) -> float:
      """

      :param pitch: input of pitch to be turned into radians
      :return: radians
      """
      return pitch * (math.pi / 180)

   def GetDistanceFromBestTag(self, cameraHeightMeters: float, targetHeightMeters: float, cameraPitchRadians: float,
                              targetPitchRadians: float) -> float:
      """

      :param cameraHeightMeters: Height of the camera in meters
      :param targetHeightMeters: Height of the wanted target in meters
      :param cameraPitchRadians: The Pitch in which the camera is facing
      :param targetPitchRadians: Input given by the camera by parsing and using the pitch method
      :return: Distance in meters from the target
      """
      return (targetHeightMeters - cameraHeightMeters) / math.tan(cameraPitchRadians + targetPitchRadians)
