package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;
import lib.MotorController.SOTAMotorController;

public class Extension extends SubsystemBase{
    private double maxLength;
    private SOTAMotorController motor;
    private DigitalInput limitswitch;
    private SuperStructureConfig config;
    private boolean hasReset = false;

    public Extension(SOTAMotorController motor, DigitalInput limitSwitch, SuperStructureConfig config){
        this.motor = motor; this.limitswitch = limitSwitch; this.config = config;
    }

    public void set(double speed){
        if(limitswitch.get() && speed < 0){
           speed = 0;
        }
        if(!hasReset && speed > 0) speed = 0;
        motor.setVoltage(speed);
      }
 

    public double getEncoder(){
        return motor.getPose();
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

    @Override
    public void periodic() {
        if(limitswitch.get()) {
            motor.getPose();
            hasReset = true;
        }
        SmartDashboard.putNumber("extensionEncoder", getEncoder());
        SmartDashboard.putNumber("Extension length", getLength());
        SmartDashboard.putBoolean("limitswitch", limitswitch.get());
    }

}
