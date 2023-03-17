package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;
import lib.Control.SOTAXboxcontroller;
import lib.MotorController.SOTAMotorController;

public class BasicIntakeCommand extends CommandBase{
    private Intake mIntake;
    private SOTAXboxcontroller controller;

    public BasicIntakeCommand(Intake mIntake, SOTAXboxcontroller controller){
        this.mIntake = mIntake; this.controller = controller;
    }

    @Override
    public void execute() {
        if(controller.getRightTriggerAxis() >= 0) mIntake.set(0.5);
        else if(controller.getRightTriggerAxis() >= 0) mIntake.set(-1);
        else mIntake.set(0);
    }

    
}
