package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm;

public class BasicArmExtension extends CommandBase{
    private Arm mArm;
    private CommandXboxController mController;

    public BasicArmExtension(Arm mArm, CommandXboxController mController){
        this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }
    @Override
    public void execute() {
        
    }
    
}
