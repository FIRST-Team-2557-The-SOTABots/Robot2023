package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class Extension extends SubsystemBase{
    private SOTAMotorController motor;
    private DigitalInput limitswitch;

    public Extension(SOTAMotorController motor, DigitalInput limitSwitch){
        this.motor = motor; this.limitswitch = limitSwitch;
    }

    public void set(double speed){
        if(limitswitch.get() && speed < 0){
           speed = 0;
           motor.resetEncoder();
           
        }
        motor.setVoltage(speed);
      }
    

    public double getEncoder(){
        return motor.getEncoder();
    }

    public double getLength(){
        return 0.0; //TODO: figure this out
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("extensionEncoder", getEncoder());
        SmartDashboard.putBoolean("limitswitch", limitswitch.get());
    }
}
