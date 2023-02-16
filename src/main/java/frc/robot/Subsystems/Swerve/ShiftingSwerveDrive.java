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
import lib.Component.GearShifter;
import lib.Component.Gyro;
import lib.Config.ShiftingSwerveDriveConfig;

public class ShiftingSwerveDrive extends SubsystemBase {
  private ShiftingSwerveModule[] mSwerveModules;
  private GearShifter mShifter;
  private Gyro mGyro;

  private Translation2d[] mModuleTranslation;
  private SwerveDriveKinematics mSwerveDriveKinematics;
  private SwerveDriveOdometry mSwerveDriveOdometry;

  private boolean mFieldCentricActive;

  private double kMaxWheelSpeed;
  private double kMaxAngularVelocity;

  /** Creates a new ShiftingSwerveDrive. */
  public ShiftingSwerveDrive(ShiftingSwerveModule[] swerveModules, GearShifter shifter, Gyro gyro, ShiftingSwerveDriveConfig config) {
    mSwerveModules = swerveModules;
    mShifter = shifter;
    mGyro = gyro;

    mModuleTranslation = config.getModuleTranslations();
    mSwerveDriveKinematics = new SwerveDriveKinematics(mModuleTranslation);
    mSwerveDriveOdometry = new SwerveDriveOdometry(
      mSwerveDriveKinematics,
      mGyro.getAngleRotation2d(),
      getModulePositions()
    );

    kMaxWheelSpeed = config.getMaxWheelSpeed();
    kMaxAngularVelocity = config.getMaxAngularVelocity();
  }

  /** 
   * Drives the drivetrain with a standard point of rotation
   * @param fwd Forward velocity
   * @param str Left velocity
   * @param rot Angular velocity
   * @param currentAngle current angle of the robot
   */
  public void drive(double fwd, double str, double rot, Rotation2d currentAngle) {
    drive(fwd, str, rot, currentAngle, new Translation2d());
  }
  
  /** 
   * Drive the drivetrain with a specified point of rotation
   * @param fwd Forward velocity
   * @param str Left velocity
   * @param rot Angular velocity
   * @param currentAngle Current angle of the robot
   * @param pointOfRotation Point the robot will rotate around 
   */
  public void drive(double fwd, double str, double rot, Rotation2d currentAngle, Translation2d pointOfRotation) {
    // Squares inputs and scales based off of max speeds
    fwd = -Math.signum(fwd) * fwd * fwd * kMaxWheelSpeed; 
    str = -Math.signum(str) * str * str * kMaxWheelSpeed;
    rot = -Math.signum(rot) * rot * rot * kMaxAngularVelocity;

    ChassisSpeeds speeds = mFieldCentricActive == true ?
      ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, currentAngle) : 
      new ChassisSpeeds(fwd, str, rot);
    SwerveModuleState[] moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds, pointOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxWheelSpeed);
    drive(ShiftingSwerveModuleState.toShiftingSwerveModuleState(moduleStates, mShifter.getGear()));
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
   * Gets the module positions from the modules
   * @return An array of the position of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    int moduleNumber = mSwerveModules.length;
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleNumber];
    for (int i = 0; i < moduleNumber; i++) {
      modulePositions[i] = mSwerveModules[i].getMeasuredPosition();
    }
    return modulePositions;
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
      getModulePositions(),
      pose
    );
  }

  /**
   * Updates the rotation of the translations of the modules
   * @param angle The angle of the drivetrain
   */
  public void updateModuleTranslation(Rotation2d angle) {
    for (int i = 0; i < mModuleTranslation.length; i++) {
      mModuleTranslation[i].rotateBy(angle);
    }
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

  /**
   * Gets the translation 2d array of the modules
   * @return Translation 2d array of the modules
   */
  public Translation2d[] getModuleTranslation(){
    return mModuleTranslation;
  }

  /**
   * Gets the rotation 2d of the drivetrain
   * @return The rotation2d of the drivetrain
   */
  public Rotation2d getRotation2d() {
    return mGyro.getAngleRotation2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose(getModulePositions(), mGyro.getAngleRotation2d());
    updateModuleTranslation(mGyro.getAngleRotation2d());
  }
}
