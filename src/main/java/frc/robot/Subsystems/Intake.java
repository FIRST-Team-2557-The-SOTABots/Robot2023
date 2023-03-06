package frc.robot.Subsystems;

import lib.MotorController.SOTAMotorController;
import lib.Subsystem.IntakeInterface;

public class Intake implements IntakeInterface{
    SOTAMotorController motors;

    public Intake(SOTAMotorController motors){
        this.motors = motors;
    }

    @Override
    public void intake() {
        motors.set(1);
        
    }

    @Override
    public void intakeCube() {
        intake();
        
    }

    @Override
    public void intakeCone() {
        intake();        
    }

    @Override
    public boolean hasPiece() {
        return false;
    }

    @Override
    public void set(double speed) {
        motors.set(speed);      
    }

    @Override
    public void stop() {
        motors.set(0);       
    }

}
