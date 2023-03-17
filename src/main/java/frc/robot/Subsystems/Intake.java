package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    MotorController motors;

    public Intake(MotorController motors){
        this.motors = motors;
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

}
