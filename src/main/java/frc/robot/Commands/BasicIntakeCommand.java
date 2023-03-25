package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;
import lib.Control.SOTA_Xboxcontroller;

public class BasicIntakeCommand extends CommandBase{
    private Intake mIntake;
    private SOTA_Xboxcontroller controller;

    public BasicIntakeCommand(Intake mIntake, SOTA_Xboxcontroller controller){
        this.mIntake = mIntake; this.controller = controller;
    }

    @Override
    public void execute() {
        // if(controller.getRightBumper()) mIntake.set(0.5);
        // else if(controller.getLeftBumper()) mIntake.set(-1);
        // else mIntake.set(0);
        // if (controller.rightTrigger().)
        mIntake.set(controller.getRightX());
        
    }

    
}
