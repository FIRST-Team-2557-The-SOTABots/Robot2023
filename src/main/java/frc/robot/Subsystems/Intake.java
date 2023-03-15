package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Intake {
    MotorController motors;

    public Intake(MotorController motors){
        this.motors = motors;
    }

    public void release(){
        set(-0.2);
    }

    
    public void intake() {
        motors.set(0.2);
        
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
