package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;

public class ExtensionPID extends CommandBase {

    public enum ExtensionSetpoint {
        RESET(-2),
        FLOOR(10),
        HIGH(39), // 32 for old claw
        MID(20),
        SUBSTATION(16);

        public double inches;

        private ExtensionSetpoint(double inches) {
            this.inches = inches;
        }
    }

    private static final double kExhaustTime = 1.5;

    private ProfiledPIDController extendPID;
    private Extension mExtension;
    private DoubleSupplier kMaxLength;
    private double mSetpoint;
    private Timer mExhaustTimer;

    public ExtensionPID(ProfiledPIDController PID, Extension mArm, DoubleSupplier maxLength){
        this.extendPID = PID; 
        this.mExtension = mArm; 
        this.kMaxLength = maxLength;
        mExhaustTimer = new Timer();
        addRequirements(mArm);
    }

    public void setSetpoint(ExtensionSetpoint setpoint) {
        this.mSetpoint = setpoint.inches;
    }

    @Override
    public void execute() {

        // setPoint = SmartDashboard.getNumber("Extension Length", 0.0);

        mSetpoint = Math.min(mSetpoint, kMaxLength.getAsDouble());
        // extendPID.reset(kSetpoint, 0.0);

        double output = extendPID.calculate(mExtension.getLengthFromStart(), mSetpoint);

        // SmartDashboard.putNumber("extensionSpeed", output);
        
        mExtension.set(output);

        SmartDashboard.putNumber("extensionGoal", mSetpoint);
        // SmartDashboard.putNumber("Max Extension", maxLength.getAsDouble());
        
    }

    public void startExhaustTimeout() {
        mExhaustTimer.restart();
    }

    public boolean exhaustTimedOut() {
        return mExhaustTimer.hasElapsed(kExhaustTime);
    }    

    
}
