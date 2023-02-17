// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveUtils;
import lib.Component.GearShifter;
import lib.Component.Gyro;

public class ShiftingSwerveDrive extends SubsystemBase {
  private ShiftingSwerveModule[] mSwerveModules;
  private GearShifter mShifter;
  private Gyro mGyro;

  private SwerveDriveKinematics mSwerveDriveKinematics;
  private SwerveDriveOdometry mSwerveDriveOdometry;
  private boolean mFieldCentricActive;

  private double kMaxWheelSpeed;

  /** Creates a new ShiftingSwerveDrive. */
  public ShiftingSwerveDrive(ShiftingSwerveModule[] swerveModules, SwerveDriveKinematics kinematics, SwerveDriveOdometry odometry, GearShifter shifter, Gyro gyro, double maxWheelSpeed) {
    mSwerveModules = swerveModules;
    mShifter = shifter;
    mGyro = gyro;
    mSwerveDriveKinematics = kinematics;
    mSwerveDriveOdometry = odometry;
    kMaxWheelSpeed = maxWheelSpeed;
  }

  /** 
   * Drives the drivetrain with a standard point of rotation
   * @param fwd Forward velocity
   * @param lft Left velocity
   * @param rot Angular velocity
   * @param currentAngle current angle of the robot
   */
  public void drive(double fwd, double lft, double rot, Rotation2d currentAngle) {
    drive(fwd, lft, rot, currentAngle, new Translation2d());
  }
  
  /** 
   * Drive the drivetrain with a specified point of rotation
   * @param fwd Forward velocity
   * @param lft Left velocity
   * @param rot Angular velocity
   * @param currentAngle Current angle of the robot
   * @param pointOfRotation Point the robot will rotate around 
   */
  public void drive(double fwd, double lft, double rot, Rotation2d currentAngle, Translation2d pointOfRotation) {
    ChassisSpeeds speeds = mFieldCentricActive == true ?
      ChassisSpeeds.fromFieldRelativeSpeeds(fwd, lft, rot, currentAngle) : 
      new ChassisSpeeds(fwd, lft, rot);
    SwerveModuleState[] moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds, pointOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxWheelSpeed);
    ShiftingSwerveModuleState[] shiftingModuleStates = ShiftingSwerveModuleState.toShiftingSwerveModuleState(moduleStates, mShifter.getGear()); 
    drive(shiftingModuleStates);
  }

  /** 
   * Drives the drivetrain with a map of SwerveModuleStates 
   * @param moduleStates map of the module states, the key corresponds to the key of the swerve module
   */
  public void drive(ShiftingSwerveModuleState[] moduleStates) {
    for (int i = 0; i < moduleStates.length; i++) {
      mSwerveModules[i].drive(moduleStates[i]);
    }
  }
  
  /** 
   * Shifts the gear of the drivetrain
   * @param gear The gear of the robot, 0 is low 1 is high 
   */
  public void shift(int gear) {
    mShifter.shift(gear);
  }



  /** 
   * Updates the pose of the robot using module positions and angle
   * @param modulePositions Positions of the module
   * @param angle Angle of the pose
   */
  public void updatePose(SwerveModulePosition[] modulePositions, Rotation2d angle) {
    mSwerveDriveOdometry.update(angle, modulePositions);
  }

  /** 
   * Updates the pose of the robot using a PathPlannerState
   * @param state State of the robot according to PathPlanner
   */
  public void updatePose(PathPlannerState state) {
    mGyro.setAngle(state.holonomicRotation.getRadians());
    Rotation2d rotation = new Rotation2d(state.holonomicRotation.getRadians());
    Pose2d pose = new Pose2d(
        state.poseMeters.getX(), 
        state.poseMeters.getY(), 
        rotation
    );
    mSwerveDriveOdometry.resetPosition(
      rotation,
      SwerveUtils.getModulePositions(mSwerveModules),
      pose
    );
  }

  /** 
   * Sets the status of field centric
   * @param fieldCentricActive The desired status of field centric
   */
  public void setFieldCentricActive(boolean fieldCentricActive) {
    mFieldCentricActive = fieldCentricActive;
  }
  
  /** 
   * Gets the status of field centric
   * @return The status of field centric 
   */
  public boolean getFieldCentricActive() {
    return mFieldCentricActive;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
