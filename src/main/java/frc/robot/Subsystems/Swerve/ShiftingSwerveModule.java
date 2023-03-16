// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.Encoder.SOTAAbsoulteEncoder;
import lib.MotorController.SOTAMotorController;

public class ShiftingSwerveModule extends SubsystemBase {

  private int mModuleNum; 

  private SOTAMotorController mAngleMotor; 
  private SOTAMotorController mSpeedMotor;

  private ProfiledPIDController mAnglePID;
  private ProfiledPIDController mSpeedPID;
  private SimpleMotorFeedforward mAngleFF;
  private SimpleMotorFeedforward mSpeedFF;

  private double[] kGearRatios;
  private double kAngleCountsPerRevolution;
  private double kSpeedCountsPerRevolution;
  private double kWheelCircumference; 

  public ShiftingSwerveModule(
    SOTAMotorController angleMotor, 
    SOTAMotorController speedMotor, 
    ShiftingSwerveModuleConfig config) {

    this.mSpeedMotor = speedMotor;
    this.mAngleMotor = angleMotor;

    this.mModuleNum = config.getModuleNum();
    this.mSpeedMotor = speedMotor; 
    this.mAngleMotor = angleMotor;

    this.kGearRatios = config.getGearRatios();

    this.kAngleCountsPerRevolution = mAngleMotor.getEncoder().getCountsPerRevolution();
    this.kSpeedCountsPerRevolution = mSpeedMotor.getEncoder().getCountsPerRevolution();
    this.kWheelCircumference = config.getWheelCircumference();

    this.mAngleFF = config.angleFF();
    this.mSpeedFF = config.speedFF();

    this.mAnglePID = config.generateAnglePID(kAngleCountsPerRevolution);
    this.mSpeedPID = config.generateSpeedPID();

  }

  /**
   * Drives the modules with a ShiftingSwerveModuleState
   * @param state The ShiftingSwerveModuleState
   */
  public void drive(ShiftingSwerveModuleState state) {
    state = ShiftingSwerveModuleState.optimize(state, getRotation2d());

    double angleSetpointNative = radiansToNative(state.angle.getRadians());
    double anglePIDOutput = mAnglePID.calculate(getAngle(), angleSetpointNative);
    double angleFFOutput = mAngleFF.calculate(mAnglePID.getSetpoint().velocity);

    mAngleMotor.setVoltage(state.speedMetersPerSecond == 0.0 ? 0.0 : angleFFOutput + anglePIDOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, kGearRatios[state.gear]);
    double speedPIDOutput = mSpeedPID.calculate(mSpeedMotor.getNativeTickVelocity(), speedSetpointNative);
    double speedFFOutput = mSpeedFF.calculate(speedSetpointNative);
    SmartDashboard.putNumber("Wheel Circumference", kWheelCircumference);
    SmartDashboard.putNumber("Speed CPR", kSpeedCountsPerRevolution);
    SmartDashboard.putNumber("Gear Ratio", kGearRatios[state.gear]);
    SmartDashboard.putNumber("Meters Per Count", getMetersPerCount(kGearRatios[state.gear]));

    mSpeedMotor.setVoltage(speedFFOutput + speedPIDOutput);
  }

  /** 
   * Gets the swerve module position of the module
   * @return The SwerveModulePosition of the module
   */
  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(
      kWheelCircumference, 
      getRotation2d()
    );
  }

  /**
   * Gets the speed of the module in meters per second
   * @return The speed of the module in meters per second
   */
  public double getSpeed(int gear) {
    return nativeToMetersPerSecond(
      mSpeedMotor.getTickVelocity(), 
      kGearRatios[gear]
    );
  }

  /** 
   * Gets the angle of the module in absolute encoder ticks
   * @return The angle of the module in absolute encoder ticks
   */
  public double getAngle() {
    return ((SOTAAbsoulteEncoder) mAngleMotor.getEncoder()).getPosition();
  }

  /**
   * Gets the angle of the absolute encoder
   * @return The angle of the absolute encoders ticks 
   */
  public double getAngleNoOffset() {
    return ((SOTAAbsoulteEncoder) mAngleMotor.getEncoder()).getPositionNoOffset();
  }

  /**
   * Gets the angle of the module in Rotation2d radians
   * @return Rotation2d of the module
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(nativeToRadians(getAngle()));
  }

  /** 
   * Converts native encoder velocity to meters per second
   * @param encoderVelocity The velocity of the encoder to convert
   * @param gearRatio The current gear ratio that the module is in
   * @return The speed of the module in meters per second
   */
  public double nativeToMetersPerSecond(double encoderVelocity, double gearRatio) {
    return encoderVelocity * 10 * getMetersPerCount(gearRatio);
  }

  /** 
   * Converts native absolute encoder counts to radians
   * @param encoderCounts Absolute encoder counts
   * @return Angle of the module in radians
   */
  public double nativeToRadians(double encoderCounts) {
    return encoderCounts * (2 * Math.PI) / kAngleCountsPerRevolution;
  }

  /**
   * Converts meters per second to native encoder counts
   * @param metersPerSecond The speed in meters per second to be converted
   * @param gearRatio The current gear ratio that the module is in
   * @return The equivalent speed motor encoder velocity in counts per 100 ms
   */
  public double metersPerSecondToNative(double metersPerSecond, double gearRatio) {
    return metersPerSecond / getMetersPerCount(gearRatio) / 10.0;
  }

  /**
   * Converts radians to native encoder ticks
   * @param radians Radians to convert
   * @return Radians converted to absolute encoder ticks
   */
  public double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * kAngleCountsPerRevolution;
  }

  /**
   * Gets the meters per encoder tick 
   * @param gearRatio Current gear ratio
   * @return The meters per count 
   */
  public double getMetersPerCount(double gearRatio) {
    return kWheelCircumference / gearRatio / kSpeedCountsPerRevolution;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
