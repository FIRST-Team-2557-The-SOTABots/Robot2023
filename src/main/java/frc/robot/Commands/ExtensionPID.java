package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;
import lib.Control.SOTA_Xboxcontroller;

public class ExtensionPID extends CommandBase {

    public enum ExtensionSetpoint {
        HIGH(32),
        MID(10),
        RESET(-2);

        public double inches;

        private ExtensionSetpoint(double inches) {
            this.inches = inches;
        }
    }

    private ProfiledPIDController extendPID;
    private Extension mExtension;
    // private SOTA_Xboxcontroller mController;
    private DoubleSupplier maxLength;
    private double setpoint;
    // private ResetExtension resetExtension;

    public ExtensionPID(ProfiledPIDController PID, Extension mArm, SOTA_Xboxcontroller mController, DoubleSupplier maxLength){
        this.extendPID = PID; 
        this.mExtension = mArm; 
        // this.mController = mController; 
        this.maxLength = maxLength;
        // this.resetExtension = new ResetExtension(mArm);
        addRequirements(mArm);
    }

    public void setSetpoint(ExtensionSetpoint newSetpoint) {
        this.setpoint = newSetpoint.inches;
    }

    @Override
    public void execute() {
        // if(mController.getA() || mController.getB() || mController.getLeftBumper() || mController.getRightBumper()) setpoint = 32;

        // if(mController.getY() || mController.getStart() || mController.getBack()) setpoint = 10;

        // if(mController.getX()) setpoint = -2;
        // setPoint = SmartDashboard.getNumber("Extension Length", 0.0);

        setpoint = Math.min(setpoint, maxLength.getAsDouble());
        extendPID.reset(setpoint, 0.0);

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
