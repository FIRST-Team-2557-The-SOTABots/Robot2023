package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;
import lib.Control.SOTA_Xboxcontroller;

public class ExtensionPID extends CommandBase{
    private ProfiledPIDController extendPID;
    private Extension mExtension;
    private SOTA_Xboxcontroller mController;
    private DoubleSupplier maxLength;
    private double setPoint;
    private ResetExtension resetExtension;

    public ExtensionPID(ProfiledPIDController PID, Extension mArm, SOTA_Xboxcontroller mController, DoubleSupplier maxLength){
        this.extendPID = PID; this.mExtension = mArm; this.mController = mController; this.maxLength = maxLength;
        this.resetExtension = new ResetExtension(mArm);
        // SmartDashboard.putNumber("maxLength2", maxLength.getAsDouble());
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        if(mController.getA() || mController.getB() || mController.getLeftBumper() || mController.getRightBumper()) setPoint = 32;

        if(mController.getY() || mController.getStart() || mController.getBack()) setPoint = 10;

        if(mController.getX()) setPoint = -2;
        // setPoint = SmartDashboard.getNumber("Extension Length", 0.0);

        setPoint = Math.min(setPoint, maxLength.getAsDouble());
        extendPID.reset(setPoint, 0.0);

        double output = extendPID.calculate(mExtension.getLengthFromStart());

        // SmartDashboard.putNumber("extensionSpeed", output);
        
        // if(mController.getLeftBumper()) setPoint = 3;
        // if(mController.getRightBumper()) setPoint = -3;
        // else setPoint = 0;
        mExtension.set(output);

        // SmartDashboard.putNumber("extensionGoal", setPoint);
        // SmartDashboard.putNumber("Max Extension", maxLength.getAsDouble());
        

        
    }

    
}
