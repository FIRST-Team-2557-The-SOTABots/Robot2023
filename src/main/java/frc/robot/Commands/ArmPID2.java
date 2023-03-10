package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Util.Controllers.SOTAXboxcontroller;

public class ArmPID2 extends CommandBase{
    private SuperStructure mArm;
    private double setpoint;
    private PIDController pidController;
    private SOTAXboxcontroller controller;

    public ArmPID2(SuperStructure mArm, PIDController pidController, double setpoint, SOTAXboxcontroller controller){
        this.mArm = mArm; this.setpoint = setpoint; this.pidController = pidController; this.controller = controller;
        addRequirements(mArm);
    }
    

    @Override
    public void execute() {
        if(controller.getA()) setpoint = 60;
        if(controller.getB()) setpoint = 150;
        pidController.setSetpoint(setpoint);
        double output = pidController.calculate(mArm.getRoll());
        if(mArm.getRotatorEncoder() > 0.587 && mArm.getRotatorEncoder() < 0.701){
            output = pidController.getSetpoint() > 90 ? 3 : -3;
        }
        mArm.setRotatorSpeed(output);
        mArm.setIntake(controller.getLeftTriggerAxis());
        SmartDashboard.putNumber("", output);
    }
    
}
