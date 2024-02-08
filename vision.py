from photonlibpy.photonCamera import PhotonCamera


class Vison:
   def __init__(self):
      self.camera = PhotonCamera("photonvision")#change this string into the actuall camera name
      result = self.camera.getLatestResult()#gets the results of the camera
      hasTargets = result.hasTargets()#checks for if the camera has targets
      targets = result.getTargets()
      target = targets[0]#firsts target in list of targets is set as the main target
      yaw = target.getYaw()
      pitch = target.getPitch()
      area = target.getArea()
      skew = target.getSkew()
      corners = target.getDetectedCorners()

      targetID = target.getFiducialId() #gets info from the target
      poseAmbiguity = target.getPoseAmbiguity()
      bestCameraToTarget = target.getBestCameraToTarget()
      alternateCameraToTarget = target.getAlternateCameraToTarget()


      self.camera.setDriverMode(True)#allows the driver to use the camera