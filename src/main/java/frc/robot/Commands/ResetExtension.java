package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;

public class ResetExtension extends CommandBase{
    private Extension mExtension; 

    public ResetExtension(Extension mExtension){
        this.mExtension = mExtension;
        addRequirements(mExtension);
    }
    @Override
    public void execute() {
        mExtension.setVoltage(-4);
        
    }
    @Override
    public void end(boolean interrupted) {
        mExtension.setVoltage(0);
    }
    @Override
    public boolean isFinished() {
        return mExtension.isFullyRetracted();
    }
}
