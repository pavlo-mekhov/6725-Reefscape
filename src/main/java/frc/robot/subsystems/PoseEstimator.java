// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class PoseEstimator extends SubsystemBase {
  VisionSubsystem vision;
  SwerveSubsystem swerve;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(VisionSubsystem vision, SwerveSubsystem swerve) {
    this.vision = vision;
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    swerve.updateOdometry();
    swerve.addVisionMeasurement(LimelightHelpers.getBotPose2d(""), 0, null);


    // This method will be called once per scheduler run
  }
}
