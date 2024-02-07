import photonlibpy.photonCamera
import photonlibpy
import wpilib.shuffleboard


class Vison:
   def __init__(self):

      self.camera = photonlibpy.photonCamera.PhotonCamera("Camera1")  # change this string into the actuall camera name
      self.board = wpilib.shuffleboard.Shuffleboard
      """
      result = self.camera.getLatestResult()  # gets the results of the camera
      hasTargets = result.hasTargets()  # checks for if the camera has targets
      targets = result.getTargets()
      target = targets[0]  # firsts target in list of targets is set as the main target
      yaw = target.getYaw()
      pitch = target.getPitch()
      area = target.getArea()
      skew = target.getSkew()
      corners = target.getDetectedCorners()

      targetID = target.getFiducialId()  # gets info from the target
      poseAmbiguity = target.getPoseAmbiguity()
      bestCameraToTarget = target.getBestCameraToTarget()
      alternateCameraToTarget = target.getAlternateCameraToTarget()

      self.camera.setDriverMode(True)  # allows the driver to use the camera
      """

      self.result = self.camera.getLatestResult()  # gets the results of the camera
      self.hasTargets = self.result.hasTargets()  # checks for if the camera has targets
      self.targets = self.result.getTargets()
      self.target = self.targets[0]  # firsts target in list of targets is set as the main target


   def getFeed(self) -> None:
      pass

   def getTargetsId(self) -> int:
      return self.target.fiducialId

   def isDetecting(self,id: int) -> bool:
      for i in self.targets:
         if i == id:
            return True
      return False

   def getTagOdometry(self,id: int) -> tuple:
      for i in self.targets:
         if i == id:
            return (i.getYaw(), i.getPitch(), i.getSkew())

   def getAllTags(self) -> tuple:
      listId = ()
      for i in self.targets:
         listId += i.getFiducialId()
      return listId

   def EnableDriverViewing(self,Enable: bool) -> None:
      self.camera.setDriverMode(Enable)
      pass

   def VideoTelemetry(self) -> None:
      self.board.addEventMarker("Tags", f"{self.getAllTags()}",wpilib.shuffleboard.ShuffleboardEventImportance.kHigh)
      pass

   def GetTagSize(self, id: int) -> float:
      for i in self.targets:
         if i.getFiducialId() == id:
            return i.getArea()
      return 10000000000
