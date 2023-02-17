// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.MotorController.SotaMotorController;

public class ShiftingSwerveModule extends SubsystemBase {

  private SotaMotorController mSpeedMotor;
  private SotaMotorController mAngleMotor;
  private AnalogInput mAngleEncoder;

  private double kAngleOffset;
  private double kAngleCountsPerRevolution;
  private double[] kGearRatios;
  private double kWheelCircumference;
  
  private ProfiledPIDController mSpeedPID;
  private SimpleMotorFeedforward mSpeedFF;
  private ProfiledPIDController mAnglePID;
  private SimpleMotorFeedforward mAngleFF;

  /** Creates a new ShiftingSwerveModule. */
  public ShiftingSwerveModule(
      SotaMotorController speedMotor, 
      SotaMotorController angleMotor, 
      AnalogInput angleEncoder,
      ShiftingSwerveModuleConfig config) {
    mSpeedMotor = speedMotor;
    mAngleMotor = angleMotor;
    mAngleEncoder = angleEncoder;
    
    kAngleOffset = config.getAngleOffset();
    kGearRatios = new double[2];
    kGearRatios[0] = config.getLoGearRatio();
    kGearRatios[1] = config.getHiGearRatio();

    kWheelCircumference = config.getWheelCircumference();

    mSpeedFF = new SimpleMotorFeedforward(
      config.getSpeedKS(),
      config.getSpeedKV()
    );
    mSpeedPID = new ProfiledPIDController(
      config.getSpeedKP(), 
      config.getSpeedKI(),
      config.getSpeedKP(),
      new TrapezoidProfile.Constraints(
        config.getSpeedMaxVel(),
        config.getSpeedMaxAccel()
      )
    );

    mAngleFF = new SimpleMotorFeedforward(
      config.getAngleKS(),
      config.getAngleKV()
    );
    mAnglePID = new ProfiledPIDController(
      config.getSpeedKP(),
      config.getAngleKI(),
      config.getAngleKD(),
      new TrapezoidProfile.Constraints(
        config.getAngleMaxVel(),
        config.getAngleMaxAccel()
      )
    );

    mAnglePID.setTolerance(config.getAnglePIDTolerance());
    mAnglePID.enableContinuousInput(0.0, config.getAngleEncoderCPR());
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

    mAngleMotor.setVoltage(state.speedMetersPerSecond == 0.0 ? 0.0 : anglePIDOutput + angleFFOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, state.gear);
    double speedPIDOutput = speedSetpointNative == 0 ? 0.0 : mSpeedPID.calculate(mSpeedMotor.getTickVelocity(), speedSetpointNative);
    double speedFFOutput = mSpeedFF.calculate(speedSetpointNative);

    mSpeedMotor.setVoltage(speedPIDOutput + speedFFOutput);
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
    return -1 * MathUtil.inputModulus(mAngleEncoder.getAverageVoltage() - kAngleOffset, 0, kAngleCountsPerRevolution) + kAngleCountsPerRevolution;
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
    return encoderCounts * 2 * Math.PI / kAngleCountsPerRevolution;
  }

  /**
   * Converts meters per second to native encoder counts
   * @param metersPerSecond The speed in meters per second to be converted
   * @param gearRatio The current gear ratio that the module is in
   * @return The equivalent speed motor encoder velocity in counts per 100 ms
   */
  public double metersPerSecondToNative(double metersPerSecond, double gearRatio) {
    return metersPerSecond / getMetersPerCount(gearRatio) / 10;
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
    return kWheelCircumference / gearRatio / mSpeedMotor.getEncoderCountsPerRevolution();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
