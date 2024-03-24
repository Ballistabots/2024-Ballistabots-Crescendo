"""
currentTime = self.Time.get()
      # aim and Shoot here
      if currentTime < 2:
         self.armPos = 36
         self.shooter_power = -1
      elif currentTime < 2.4:
         self.intake_speed = 0.3

      elif currentTime < 2.8:
         self.armPos = 64.66
         self.shooter_power = 0

      else:
         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)
         self.intake_speed = 0
         self.shooter_power = 0
         self.armPos = 12.4
"""