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


      elif currentTime < 3.5:
         self.intake_speed = 0.3
         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.3, -0.01, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)

      elif currentTime < 4:

         self.armPos = 55
         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.3, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)



      elif currentTime < 5:

         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)



      elif currentTime < 6.5:
         self.intake_speed = -0.05
         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.4, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)
         self.armPos = 36


      elif currentTime < 7:
         self.shooter_power = -1

         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)

         self.intake_speed = 1

      else:
         self.AutoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,
                                                                 Rotation2d.fromDegrees(self.drivetrain.getGyro()))
         self.drivetrain.driveFromChassisSpeeds(self.AutoSpeeds)
         self.intake_speed = 0
         self.shooter_power = 0
         self.armPos = 12.4

      # fucntions for setting arm poses and shooter speeds
      self.arm.moveToEncoderPos(self.armPos)
      self.Shooter.Outtake(self.shooter_power)
      self.Shooter.SetIntakePower(self.intake_speed)

"""