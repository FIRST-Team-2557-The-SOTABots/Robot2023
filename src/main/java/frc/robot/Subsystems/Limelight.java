// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable mTable;

  /** Creates a new Limelight. */
  public Limelight() {
    mTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public Pose2d getPredictedBotPose2d() {
    double[] botpose = mTable.getEntry("botpose").getDoubleArray(new double[6]); 
    return new Pose2d(botpose[0], botpose[1], new Rotation2d(Math.toRadians(botpose[3])));
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
