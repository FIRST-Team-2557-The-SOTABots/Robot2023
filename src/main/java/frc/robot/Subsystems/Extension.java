package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
        this.motor = motor; this.limitswitch = limitSwitch; this.maxLength = config.getMaxExtension(); this.config = config;
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
    public double getMaxExtension(){
        return maxLength;
    }

    @Override
    public void periodic() {
        if(limitswitch.get()) {
            motor.getEncoder().reset();;
            hasReset = true;
        }
        // SmartDashboard.putNumber("extensionEncoder", getEncoder());
        // SmartDashboard.putNumber("Extension length inches", getLength());
        // SmartDashboard.putBoolean("limitswitch", limitswitch.get());
        // SmartDashboard.putNumber("extension motor limit", motor.getMotorLimits().getUpperLimit());

    }

}
