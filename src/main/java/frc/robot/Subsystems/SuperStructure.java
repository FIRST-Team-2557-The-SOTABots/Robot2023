// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;

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
  private double rotationEncoderOffset;
  private double delta;

  /** Creates a new ArmSubsystem. */
  public SuperStructure(DoubleSupplier extensionLength, DoubleSupplier CurrentAngle, SuperStructureConfig config) {
    this.extensionLength = extensionLength; this.CurrentAngle = CurrentAngle;
     this.height = config.getHeight(); this.bOffset = config.getbOffset(); 
     this.fOffset = config.getfOffset(); this.armLength = config.getArmBaseLength();
     this.fAbsoluteOffset = config.getfAbsoluteOffset(); this.bAbsoluteOffset = config.getbAbsoluteOffset();
     this.maxExtension = config.getMaxExtension();

  }

  public double maxExtension(){
    if(CurrentAngle.getAsDouble() >= 90 && CurrentAngle.getAsDouble() <= 270){
      return maxExtension;
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

  public double getRotationEncoderOffset(){
    return rotationEncoderOffset;
}
public double getDelta(){
    return delta;
}
 
  
}
