package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;

public class ExtensionPID extends CommandBase {

    public enum ExtensionSetpoint {
        RESET(-0.5),
        REST(-0.5),
        FLOOR(10),
        FLOORCONE(17),
        HIGH(38), // 32 for old claw
        MID(20.25),
        SUBSTATION(11),
        SINGLE(0.0);

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
    private double mThrottle;
    private Timer mExhaustTimer;

    public ExtensionPID(ProfiledPIDController PID, Extension mArm, DoubleSupplier maxLength) {
        this.extendPID = PID; 
        this.mExtension = mArm; 
        this.kMaxLength = maxLength;
        mExhaustTimer = new Timer();
        mThrottle = 1.0; // woopsies NaN error
        addRequirements(mArm);
    }

    public void setSetpoint(ExtensionSetpoint setpoint) {
        this.mSetpoint = setpoint.inches;
    }

    public boolean atSetpoint(){
        if(mSetpoint < 0 && mExtension.getLengthFromStart() == 0) return true; 
        return Math.abs(mSetpoint - mExtension.getLengthFromStart()) < 2;
    }

    public void execute() {

        // setPoint = SmartDashboard.getNumber("Extension Length", 0.0);

        double adjustedSetpoint = Math.min(mSetpoint, kMaxLength.getAsDouble());
        // extendPID.reset(kSetpoint, 0.0);

        double output = extendPID.calculate(mExtension.getLengthFromStart(), adjustedSetpoint);

        // SmartDashboard.putNumber("extensionSpeed", output);
        
        mExtension.setVoltage(output);

        // SmartDashboard.putNumber("extensionGoal", mSetpoint);
        // SmartDashboard.putNumber("Max Extension", maxLength.getAsDouble());
        
    }

    public void startExhaustTimeout() {
        mExhaustTimer.restart();
    }

    public boolean exhaustTimedOut() {
        return mExhaustTimer.hasElapsed(kExhaustTime);
    }   
    
    // TODO: finish
    public void throttleExtension(double percent) {
        mThrottle = 1.0 - Math.abs(percent);
    }

    
}
