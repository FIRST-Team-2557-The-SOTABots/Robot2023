// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Configs.MotorControllerConfig;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;
import frc.robot.Util.Configs.SuperStructureConfig;

public class SuperStructure extends SubsystemBase {

  // private SOTAGyro pigeon;
  // private SOTAMotorController rotatorMotor;
  // private SOTAMotorController winchMotor;
  // private DigitalInput limitSwitch;
  // private MotorController intakeMotors;

  private DoubleSupplier extensionLength;
  private DoubleSupplier CurrentAngle;
  private double height;
  private double bOffset;
  private double fOffset;
  private double armLength;
  private double fAbsoluteOffset;
  private double bAbsoluteOffset;
  private double maxExtension;

  /** Creates a new ArmSubsystem. */
  public SuperStructure(DoubleSupplier extensionLength, DoubleSupplier CurrentAngle, SuperStructureConfig config) {
    this.extensionLength = extensionLength; this.CurrentAngle = CurrentAngle;
     this.height = config.getHeight(); this.bOffset = config.getbOffset(); 
     this.fOffset = config.getfOffset(); this.armLength = config.getArmBaseLength();
     this.fAbsoluteOffset = config.getfAbsoluteOffset(); this.bAbsoluteOffset = config.getbAbsoluteOffset();
     this.maxExtension = config.getMaxExtension();

  }

  public double maxExtension(){
    SmartDashboard.putNumber("angle", CurrentAngle.getAsDouble());
    if(CurrentAngle.getAsDouble() >= 90 && CurrentAngle.getAsDouble() <= 270){
      return 31;
    }
    double heightOffset = CurrentAngle.getAsDouble() < 180 ? fAbsoluteOffset: bAbsoluteOffset; 
    return MathUtil.clamp((((height + heightOffset) / (Math.cos(Math.toRadians(CurrentAngle.getAsDouble())))) - armLength), 0, this.maxExtension); 
  }

  //GOOD
  public double minRotation(){
    double heightOffset = fOffset;
    return Math.max(Math.toDegrees(Math.acos((height + heightOffset) / extensionLength.getAsDouble())), 44);
  }

  public double maxRotation(){
    double heightOffset = bOffset;
    return Math.min((360 - Math.toDegrees(Math.acos((height + heightOffset) / extensionLength.getAsDouble()))), 297);
  }

  // public double getRoll(){
  //   if(rotatorMotor.getEncoder() > 0.653){
  //     return 180 - pigeon.getRoll();
  //   }   //TODO: put in config
  //   return pigeon.getRoll();
  // }

  // public void setRotatorSpeed(double spd){
  //     rotatorMotor.set(MathUtil.clamp(spd, -0.5, 0.5));
    
  // }

  // public double getRotatorEncoder(){
  //   return rotatorMotor.getEncoder();
  // }

  // public void setExtensionSpeed(double speed){
  //   if(limitSwitch.get() && speed < 0){
  //      speed = 0;
  //      winchMotor.resetEncoder();
       
  //   }
  //   winchMotor.setVoltage(speed);
  // }

  // public void setIntake(double voltage){
  //   SmartDashboard.putNumber("IntakeSpeed", voltage);
  //   intakeMotors.set(voltage); //TODO: make it set
  // }

  // public Pose3d getClawPose3d(){
  //   return new Pose3d(new Translation3d(), new Rotation3d());
  // }

  // public double getExtension(){
  //   return winchMotor.getEncoder();
  // }

  // @Override
  // public void periodic() {
  //   // SmartDashboard.putNumber("encoder count: ", rotatorMotor.getEncoder());
  //   // SmartDashboard.putNumber("Pigeon Yaw: ", pigeon.getYaw());
  //   // SmartDashboard.putNumber("Pigeon Pitch: ", pigeon.getPitch());
  //   // SmartDashboard.putNumber("Pigeon Roll: ", pigeon.getRoll());
  //   // SmartDashboard.putNumber("Motor Speed:", rotatorMotor.get());
  //   // SmartDashboard.putBoolean("at Upper Limit", rotatorMotor.atUpperLimit());
  //   // SmartDashboard.putBoolean("At lower limit", rotatorMotor.atLowerLimit());
  //   // SmartDashboard.putNumber("NEO encoder", rotatorMotor.getTickPosition());
  //   SmartDashboard.putNumber("Arm Extension:", winchMotor.getEncoder());
  //   SmartDashboard.putNumber("Extension power", winchMotor.get());
  //   SmartDashboard.putBoolean("limitSwitch", limitSwitch.get());
  // }
  public boolean checkLimits(double angleCount, double winchCount, double offsetOffGround, SuperStructureConfig config){
    double absoluteZero = config.getEncoderAtZeroDegrees();
    double baseLength = config.getArmBaseLength();
    double encoderPerInch = config.getEncoderPerInch();
    double encoderPerAngle = config.getEncoderPerDegree();
    double height = config.getHeight();
    double length = baseLength + (winchCount/encoderPerInch);
    // double angle = getAngle(angleCount, absoluteZero, encoderPerAngle);
    // if(angle => 90 && angle <= 270){
      return false;
    // }
    // if(angle < 90){
      
    // }else{

    // }
    // return false;
  }
  
}
