package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class AutoLevel extends CommandBase{
    private ShiftingSwerveDrive mSwerveDrive;
    
    
    public AutoLevel(ShiftingSwerveDrive mShiftingSwerveDrive){
        this.mSwerveDrive = mShiftingSwerveDrive;
        addRequirements(mSwerveDrive);
    }

    @Override
    public void execute() {
        mSwerveDrive.setFieldCentricActive(false);
        double output = MathUtil.clamp(mSwerveDrive.getPitch() / 200, 0.2, 0.2);
        mSwerveDrive.drive(output,0,0, mSwerveDrive.getRotation2d());
    }
    @Override
    public boolean isFinished() {
        return Math.abs(mSwerveDrive.getPitch()) < 0.5;
    }
    
}
