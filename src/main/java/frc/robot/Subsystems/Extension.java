package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;
import lib.MotorController.SOTA_MotorController;

public class Extension extends SubsystemBase{
    private SOTA_MotorController mMotor;
    private DigitalInput mLimitswitch;
    private double kMaxLength;
    private double kArmBaseLength;
    private double kEncoderPerInch;
    private boolean hasReset = false;

    public Extension(SOTA_MotorController motor, DigitalInput limitSwitch, SuperStructureConfig config){
        this.mMotor = motor; 
        this.mLimitswitch = limitSwitch; 
        this.kMaxLength = config.getMaxExtension();
        this.kArmBaseLength = config.getArmBaseLength();
        this.kEncoderPerInch = config.getEncoderPerInch();
    }

<<<<<<< HEAD
    public void set(double speed){
        SmartDashboard.putNumber("SpeedInit", speed);
        if(limitswitch.get() && speed < 0){
           speed = 0;
        }
        if(!hasReset && speed > 0) speed = 0;
        motor.setVoltage(speed);
        SmartDashboard.putNumber("extensionSpeed", speed);
      }
=======
    public void setVoltage(double voltage){
        if(mLimitswitch.get() && voltage < 0) {
           voltage = 0;
        }
        if(!hasReset && voltage > 0) voltage = 0;
        mMotor.setVoltage(voltage);
        SmartDashboard.putNumber("extensionSpeed", voltage);
    }
>>>>>>> fcc58b6 (Changed some config components to be grabbed in constructor. dont make config object a member variable please)
 

    public double getEncoder(){
        return mMotor.getEncoderPosition();
    }

    public double getLength(){
<<<<<<< HEAD
        return config.getArmBaseLength() + getLengthFromStart();
    }   

    public double getLengthFromStart() {
        return Math.max((getEncoder()/config.getEncoderPerInch()),0);
=======
        return kArmBaseLength + (getEncoder() / kEncoderPerInch);
    }   

    public double getLengthFromStart() {
        return (getEncoder() / kEncoderPerInch);
>>>>>>> fcc58b6 (Changed some config components to be grabbed in constructor. dont make config object a member variable please)
    }

    public boolean isFullyRetracted(){
        return mLimitswitch.get();
    }
    public double getMaxExtension(){
        return kMaxLength;
    }

    @Override
    public void periodic() {
        if(mLimitswitch.get()) {
            mMotor.resetNativeEncoder();
            hasReset = true;
        }
        // SmartDashboard.putNumber("extensionEncoder", getEncoder());
<<<<<<< HEAD
        // SmartDashboard.putNumber("Extension length inches", getLength());
        // SmartDashboard.putBoolean("limitswitch", limitswitch.get());
=======
        // SmartDashboard.putNumber("length", getLengthFromStart());
        // SmartDashboard.putNumber("Extension length inches", getLength());
        // SmartDashboard.putBoolean("limitswitch", mLimitswitch.get());
>>>>>>> fcc58b6 (Changed some config components to be grabbed in constructor. dont make config object a member variable please)
        // SmartDashboard.putNumber("extension motor limit", motor.getMotorLimits().getUpperLimit());
        SmartDashboard.putNumber("Extension get", mMotor.get());

    }

}
