package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Configs.ShiftingSwerveModuleConfig;
import frc.robot.Util.Controllers.CompositeMotor;
import frc.robot.Util.Interfaces.SOTAMotorController;
import frc.robot.Util.Interfaces.ShiftingSwerveModuleInterface;

public class ShiftingSwerveModuleI2 extends SubsystemBase implements ShiftingSwerveModuleInterface{


    private SOTAMotorController mAngleMotor;
    private SOTAMotorController mSpeedMotor;

    private ProfiledPIDController mAnglePID;
    private ProfiledPIDController mSpeedPID;
    private SimpleMotorFeedforward mAngleFF;
    private SimpleMotorFeedforward mSpeedFF;

    private double kAngleOffset;
    private double kAngleCountsPerRevolution;
    private double[] kGearRatios;
    private double kWheelCircumference; 

    private int modulePosition; 

    public ShiftingSwerveModuleI2( int modulePosition,
      SOTAMotorController angleMotor, 
            SOTAMotorController speedMotor, 
            ShiftingSwerveModuleConfig config) {

        this.modulePosition = modulePosition;
        this.mSpeedMotor = speedMotor; this.mAngleMotor = angleMotor;

        this.kAngleOffset = config.getAngleOffset();
        this.kGearRatios = config.getGearRatios();

        this.kAngleCountsPerRevolution = config.getAngleEncoderCPR();
        this.kWheelCircumference = config.getWheelCircumference();

        this.mAngleFF = config.angleFF();
        this.mSpeedFF = config.speedFF();

        this.mAnglePID = config.anglePID();
        this.mSpeedPID = config.speedPID();
        
        
    }

//     /**
//    * Drives the modules with a ShiftingSwerveModuleState
//    * @param state The ShiftingSwerveModuleState
//    */
  public void drive(ShiftingSwerveModuleState state) {
    state = ShiftingSwerveModuleState.optimize(state, getRotation2d());

    double angleSetpointNative = radiansToNative(state.angle.getRadians());
    double anglePIDOutput = mAnglePID.calculate(getAngle(), angleSetpointNative);
    double angleFFOutput = mAngleFF.calculate(mAnglePID.getSetpoint().velocity);

    mAngleMotor.setVoltage(state.speedMetersPerSecond == 0.0 ? 0.0 : anglePIDOutput + angleFFOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, kGearRatios[state.getGear()]);
    double speedPIDOutput = speedSetpointNative == 0 ? 0.0 : mSpeedPID.calculate(mSpeedMotor.getSensorTickVelocity(), speedSetpointNative);
    double speedFFOutput = mSpeedFF.calculate(speedSetpointNative);

    mSpeedMotor.setVoltage(speedPIDOutput + speedFFOutput);
    
    SmartDashboard.putNumber("speed setpoint " + modulePosition, state.speedMetersPerSecond);
    SmartDashboard.putNumber("angle setpoint " + modulePosition, state.angle.getRadians());
    SmartDashboard.putNumber("angleVoltage" + modulePosition, state.speedMetersPerSecond == 0.0 ? 0.0 : anglePIDOutput + angleFFOutput);
    SmartDashboard.putNumber("speedVoltage" + modulePosition, speedFFOutput + speedPIDOutput);

    
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
      mSpeedMotor.getSensorTickVelocity(), 
      kGearRatios[gear]
    );
  }

  /** 
   * Gets the angle of the module in absolute encoder ticks
   * @return The angle of the module in absolute encoder ticks
   */
  public double getAngle() {
    return -1.0 * MathUtil.inputModulus(mAngleMotor.getEncoder() - kAngleOffset, 0, kAngleCountsPerRevolution) + kAngleCountsPerRevolution;
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
    SmartDashboard.putNumber("Angle no offset " + modulePosition, getAngle());
  }
}
