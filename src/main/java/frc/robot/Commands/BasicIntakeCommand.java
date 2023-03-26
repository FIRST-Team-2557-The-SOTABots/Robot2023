package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;
import lib.Control.SOTA_Xboxcontroller;

public class BasicIntakeCommand extends CommandBase{
    private Intake mIntake;
    private CommandXboxController mController;

    public BasicIntakeCommand(Intake mIntake, SOTA_Xboxcontroller controller){
        this.mIntake = mIntake; 
        this.mController = controller;
        addRequirements(mIntake);
    }

    @Override
    public void execute() {
        double speed = mController.getLeftY() * 0.5;

        mIntake.set(speed);
    }

    
}
