package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Util.Interfaces.IntakeInterface;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class Intake implements IntakeInterface{
    MotorController motors;

    public Intake(MotorController motors){
        this.motors = motors;
    }

    public void release(){
        set(-0.2);
    }

    @Override
    public void intake() {
        motors.set(0.2);
        
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
