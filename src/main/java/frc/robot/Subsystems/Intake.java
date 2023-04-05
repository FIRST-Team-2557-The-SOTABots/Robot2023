package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.MotorController.SOTA_MotorController;

public class Intake extends SubsystemBase {
    private final SOTA_MotorController mTopMotor;
    private final SOTA_MotorController mBottomMotor;

    public Intake(SOTA_MotorController topMotor, SOTA_MotorController bottomMotor){
        mTopMotor = topMotor;
        mBottomMotor = bottomMotor;
    }

    public void set(double speed) {
        mTopMotor.set(speed);
        mBottomMotor.set(speed);
    }

    public void intakeCube(double speed) {
        mTopMotor.set(speed);
        mBottomMotor.set(speed);
    }
    
    public void intakeCone(double speed) {
        mTopMotor.set(-speed);
        mBottomMotor.set(speed);
    }
    
    public boolean hasPiece() {
        return false;
    }
    public void outTakeCone(){
        intakeCone(-1);
    }

    public void outTakeCube(){
        intakeCube(-0.5);
    }
    public void intakeCone(){
        intakeCone(0.2);
    }
    public void intakeCube(){
        intakeCube(0.5);
    }

    
    public void stop() {
        mTopMotor.set(0);
        mBottomMotor.set(0);       
    }

    @Override
    public void periodic() {
    }

}
