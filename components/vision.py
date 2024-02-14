import photonlibpy
import photonlibpy.photonCamera
import wpilib.shuffleboard


class Vison:
   def __init__(self):

      self.camera = photonlibpy.photonCamera.PhotonCamera("Camera1")  # change this string into the actuall camera name
      self.board = wpilib.shuffleboard.Shuffleboard

      self.aprilTags = (0,
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
      #self.target = self.targets[None]  # firsts target in list of targets is set as the main target

   def getFeed(self) -> None:
      pass

   def isDetecting(self, id: int) -> bool:
      if len(self.targets) <= 0:
         pass
      for i in self.targets:
         if i == id:
            return True
      return False

   def getTagOdometry(self, id: int) -> tuple:

      if len(self.targets) <= 0:
         pass
      for i in self.targets:
         if i == id:
            return (i.getYaw(), i.getPitch(), i.getSkew())

   def getAllTags(self) -> tuple:
      if len(self.targets) <= 0 :
         pass
      listId = ()
      for i in self.targets:
         listId += i.getFiducialId()
      return listId

   def EnableDriverViewing(self, Enable: bool) -> None:
      if len(self.targets) <= 0 :
         pass
      self.camera.setDriverMode(Enable)
      pass

   def VideoTelemetry(self) -> None:
      if len(self.targets) <= 0 :
         pass
      self.board.addEventMarker("Tags", f"{self.getAllTags()}", wpilib.shuffleboard.ShuffleboardEventImportance.kHigh)
      pass

   def GetTagSize(self, id: int) -> float:
      if len(self.targets) <= 0 :
         pass
      for i in self.targets:
         if i.getFiducialId() == id:
            return i.getArea()
      return 10000

   def getResults(self):
      return self.camera.getLatestResult()
