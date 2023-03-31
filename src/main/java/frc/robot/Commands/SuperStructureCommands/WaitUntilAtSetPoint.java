package frc.robot.Commands.SuperStructureCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WaitUntilAtSetPoint extends WaitUntilCommand{


    public WaitUntilAtSetPoint(BooleanSupplier condition) {
        super(condition);
    }

   

    
}
