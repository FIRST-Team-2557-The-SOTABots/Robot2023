package frc.robot.Subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    MotorController motors;
    ColorSensorV3 mSenseor;

    public Intake(MotorController motors, ColorSensorV3 cV3){
        this.motors = motors;
        this.mSenseor = cV3;
    }

    public void release(){
        set(-0.25);
    }

    
    public void intake() {
        set(0.3);
        
    }

    
    public void intakeCube() {
        intake();
        
    }

    
    public void intakeCone() {
        intake();        
    }

    
    public boolean hasPiece() {
        return false;
    }

    
    public void set(double speed) {
        motors.set(speed);      
    }

    
    public void stop() {
        motors.set(0);       
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("color sensor", mSenseor.getProximity());
    }

}
