package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class AutoLevel extends CommandBase{
    private ShiftingSwerveDrive mSwerveDrive;
    private PIDController mPitchController;
    private PIDController mRollController;
    private boolean prevCentricState;
    
    
    public AutoLevel(ShiftingSwerveDrive mShiftingSwerveDrive){
        this.mSwerveDrive = mShiftingSwerveDrive;
        mPitchController = new PIDController(0.05, 0.1, 0);
        mPitchController.setIntegratorRange(-0.05, 0.05);
        mRollController = new PIDController(0.05, 0.1, 0);
        mRollController.setIntegratorRange(-0.05, 0.05);
        prevCentricState = mSwerveDrive.getFieldCentricActive();
        addRequirements(mSwerveDrive);
    }

    @Override
    public void execute() {
        mSwerveDrive.setFieldCentricActive(false);
        double pitchOutput = mPitchController.calculate(mSwerveDrive.getPitch(), 0.0);
        double rollOutput = mRollController.calculate(mSwerveDrive.getRoll(), 0.0);

        mSwerveDrive.drive(
            new ChassisSpeeds(
                -Math.signum(pitchOutput) * pitchOutput * pitchOutput,
                Math.signum(rollOutput) * rollOutput * rollOutput,
                0.0
            )
        );

    }
    @Override
    public void end(boolean interrupted) {
        mSwerveDrive.setFieldCentricActive(prevCentricState);
    }


    @Override
    public boolean isFinished() {
        return Math.abs(mSwerveDrive.getPitch()) < 1 && Math.abs(mSwerveDrive.getRoll()) < 1;
    }
    
}
