// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;


import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.MotorController.SOTA_MotorController;

public class ShiftingSwerveModule extends SubsystemBase {

  private String mModulePosition; 

  private SOTA_MotorController mAngleMotor; 
  private SOTA_MotorController mSpeedMotor;

  private ProfiledPIDController mAnglePID;
  private PIDController mSpeedPID;
  private SimpleMotorFeedforward mAngleFF;
  private SimpleMotorFeedforward mSpeedFF;

  private double[] kGearRatios;
  private double kAngleCountsPerRevolution;
  private double kSpeedCountsPerRevolution;
  private double kWheelCircumference; 

  private IntSupplier mGear;

  private double mPrevEncoderPosition;
  private double mDistance;

  public ShiftingSwerveModule(
    SOTA_MotorController angleMotor, 
    SOTA_MotorController speedMotor, 
    IntSupplier gear,
    ShiftingSwerveModuleConfig config) {

    
    this.mModulePosition = config.getModulePosition();
    
    this.mSpeedMotor = speedMotor; 
    this.mAngleMotor = angleMotor;

    this.mGear = gear;

    this.kGearRatios = config.getGearRatios();

    this.kAngleCountsPerRevolution = mAngleMotor.getEncoder().getCountsPerRevolution();
    this.kSpeedCountsPerRevolution = mSpeedMotor.getNativeCountsPerRevolution();
    this.kWheelCircumference = config.getWheelCircumference();

    // this.mOffsets = new InterpolatingSwerveOffsetTreeMap(config.getAnglesToOffset(), kAngleCountsPerRevolution);

    this.mAngleFF = config.angleFF();

    this.mSpeedFF = config.speedFF();

    this.mAnglePID = config.generateAnglePID(kAngleCountsPerRevolution);
    this.mSpeedPID = config.generateSpeedPID();

    

    // SmartDashboard.putNumber("Voltage" + mModulePosition, 0);


  }

  /**
   * Drives the modules with a ShiftingSwerveModuleState
   * @param state The ShiftingSwerveModuleState
   */
  public void drive(ShiftingSwerveModuleState state) { // TODO: Change back to SwerveModuleState
    state = ShiftingSwerveModuleState.optimize(state, getRotation2d());

    double angleSetpointNative = Math.abs(radiansToNative(state.angle.getRadians()));
    double anglePIDOutput = mAnglePID.calculate(getAngle(), angleSetpointNative);
    double angleFFOutput = mAngleFF.calculate(mAnglePID.getSetpoint().velocity);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, kGearRatios[mGear.getAsInt()]);
    double speedPIDOutput = mSpeedPID.calculate(mSpeedMotor.getNativeEncoderVelocity(), speedSetpointNative);
    double speedFFOutput = mSpeedFF.calculate(speedSetpointNative);

    // double error =  getAngle() - angleSetpointNative;
    // SmartDashboard.putNumber("angle error" + mModulePosition,error);
    
    double error = speedSetpointNative - mSpeedMotor.getNativeEncoderVelocity();
    SmartDashboard.putNumber("speed error " + mModulePosition, error);


    mAngleMotor.setVoltage(speedSetpointNative == 0 ? 0 :  angleFFOutput + anglePIDOutput);
    mSpeedMotor.setVoltage(speedFFOutput + speedPIDOutput);


  }

  /** 
   * Gets the swerve module position of the module
   * @return The SwerveModulePosition of the module
   */
  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(
      getDistance(),
      getRotation2d()
    );
  }

  /**
   * To accurately track the behavior of the drivetrain, knowing the actual state of the module is necessary since set state
   * often varies from actual state.
   * @return the measured state of the swerve module (not set state[!)
   */
  public SwerveModuleState getMeasuredState() {
    SwerveModuleState state = new SwerveModuleState(mSpeedMotor.getEncoderVelocity(), getRotation2d());
    if (state.speedMetersPerSecond < 0.0) {
      state.speedMetersPerSecond *= -1;
      state.angle = state.angle.plus(new Rotation2d(Math.PI));
    } 
    return state;
  }

  
  /**
   * Gets the speed of the module in meters per second
   * @return The speed of the module in meters per second
   */
  public double getSpeed(int gear) {
    return nativeToMetersPerSecond(
      mSpeedMotor.getEncoderVelocity(), 
      kGearRatios[gear]
    );
  }

  /**
   * Gets total distance the module has traveled
   * @return The distance traveled
   */
  public double getDistance() {
    return mDistance;
  }

  /**
   * Updates the total distance with the previous encoder position
   */
  public void updateDistance() {
    mDistance += (mSpeedMotor.getNativeEncoderPosition() - mPrevEncoderPosition) * (kWheelCircumference / (kGearRatios[getCurrentGear()] * kSpeedCountsPerRevolution));
    this.mPrevEncoderPosition = mSpeedMotor.getNativeEncoderPosition();
  }

  public boolean shouldShift(){
    return nativeToMetersPerSecond(mSpeedMotor.getNativeEncoderVelocity(), kGearRatios[mGear.getAsInt()]) > 1.8; //TODO dont hard code
  }

  /**
   * Gets the gear from the ShiftingSwerveDrive
   */
  public int getCurrentGear(){
    return mGear.getAsInt();
  }

  /** 
   * Gets the angle of the module in absolute encoder ticks
   * @return The angle of the module in absolute encoder ticks
   */
  public double 
  getAngle() {
    // return mOffsets.nativeToAdjusted(getAngleNoOffset());
    return  mAngleMotor.getEncoderPosition();//TODO: I think that this needs to be inverted
  }

  /**
   * Gets the angle of the absolute encoder
   * @return The angle of the absolute encoders ticks 
   */
  public double getAngleNoOffset() {
    return  mAngleMotor.getEncoder().getAbsolutePosition();
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
    // return mOffsets.getInterpolated(radians);
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
    
    updateDistance();
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("isInverted" + mModulePosition, mSpeedMotor.getInverted());

    boolean shouldShift = Math.abs(mSpeedMotor.get()) > 0.70;
    
    SmartDashboard.putNumber("angle no offset " + mModulePosition, mAngleMotor.getEncoder().getAbsolutePosition());
    SmartDashboard.putBoolean("Should shift", shouldShift);

    // SmartDashboard.putNumber("Speed Motor Speed", mSpeedMotor.getNativeEncoderVelocity());
    
    // SmartDashboard.putNumber("Angle error" + mModulePosition, mSpeedPID.getVelocityError());

    // SmartDashboard.putNumber("distance" + mModulePosition, mDistance);
    // SmartDashboard.putNumber("current draw" + mModulePosition, mSpeedMotor.getMotorCurrent());
    // SmartDashboard.putNumber("mModulePosition", kAngleCountsPerRevolution);
  }
}
