// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private enum Pipeline {
    LIGHTS_OFF,
    APRILTAG_ODOMETRY;

  }
  
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  /** Creates a new Limelight. */
  public Limelight() {}

  public Pose3d getRobotPoseFromAprilTag() {
    setPipline(Pipeline.APRILTAG_ODOMETRY);
    double[] poseCoords = table.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(
      poseCoords[0],
      poseCoords[1],
      poseCoords[2],
      new Rotation3d(
        poseCoords[3],
        poseCoords[4],
        poseCoords[5]
      )
    );
  }

  private void setPipline(Pipeline desiredPipeline) {
    table.getEntry("pipeline").setNumber(desiredPipeline.getId());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
