package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;
import lib.MotorController.SOTA_MotorController;

public class Extension extends SubsystemBase{
    private double maxLength;
    private SOTA_MotorController motor;
    private DigitalInput limitswitch;
    private SuperStructureConfig config;
    private boolean hasReset = false;

    public Extension(SOTA_MotorController motor, DigitalInput limitSwitch, SuperStructureConfig config){
        this.motor = motor; this.limitswitch = limitSwitch; this.maxLength = config.getMaxExtension(); this.config = config;
    }

    public void set(double speed){
        SmartDashboard.putNumber("SpeedInit", speed);
        if(limitswitch.get() && speed < 0){
           speed = 0;
        }
        if(!hasReset && speed > 0) speed = 0;
        // motor.setVoltage(speed);
        SmartDashboard.putNumber("extensionSpeed", speed);
      }
 

    public double getEncoder(){
        return motor.getEncoderPosition();
    }

    public double getLength(){
        return config.getArmBaseLength() + (getEncoder()/config.getEncoderPerInch());
    }   

    public double getLengthFromStart() {
        return (getEncoder()/config.getEncoderPerInch());
    }

    public boolean isFullyRetracted(){
        return limitswitch.get();
    }
    public double getMaxExtension(){
        return maxLength;
    }

    @Override
    public void periodic() {
        if(limitswitch.get()) {
            motor.resetNativeEncoder();
            hasReset = true;
        }
        // SmartDashboard.putNumber("extensionEncoder", getEncoder());
        // SmartDashboard.putNumber("Extension length inches", getLength());
        // SmartDashboard.putBoolean("limitswitch", limitswitch.get());
        // SmartDashboard.putNumber("extension motor limit", motor.getMotorLimits().getUpperLimit());

    }

}
