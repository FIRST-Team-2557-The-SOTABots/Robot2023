// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Encoder.SOTAEncoder;
import frc.robot.util.Gyro.SOTAGyro;
import frc.robot.util.MotorController.SOTAMotorController;

public class SuperStructure extends SubsystemBase {

  private SOTAGyro pigeon;
  private SOTAMotorController rotatorMotor;
  private SOTAMotorController winchMotor;
  private DigitalInput limitSwitch;
  private SOTAMotorController intakeMotors;

  /** Creates a new ArmSubsystem. */
  public SuperStructure(SOTAGyro pigeon, SOTAMotorController winchMotor,
   SOTAMotorController rotatorMotor, DigitalInput limitSwitch,
    SOTAMotorController intake) {
    this.pigeon = pigeon;
    this.winchMotor = winchMotor;
    this.rotatorMotor = rotatorMotor;
    this.limitSwitch = limitSwitch;
    this.intakeMotors = intake;
  }

  public double getRoll(){
    return pigeon.getRoll();
  }

  public void setRotatorSpeed(double spd){
      rotatorMotor.set(MathUtil.clamp(spd, -0.5, 0.5));
    
  }

  public void setExtensionSpeed(double speed){
    if(limitSwitch.get() && speed < 0) winchMotor.set(0);
    winchMotor.setVoltage(speed);
  }

  public void setIntake(double speed){
    intakeMotors.setVoltage(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder count: ", rotatorMotor.getEncoder());
    SmartDashboard.putNumber("Pigeon Yaw: ", pigeon.getYaw());
    SmartDashboard.putNumber("Pigeon Pitch: ", pigeon.getPitch());
    SmartDashboard.putNumber("Pigeon Roll: ", pigeon.getRoll());
    SmartDashboard.putNumber("Motor Speed:", rotatorMotor.get());

    // This method will be called once per scheduler run
  }
}
