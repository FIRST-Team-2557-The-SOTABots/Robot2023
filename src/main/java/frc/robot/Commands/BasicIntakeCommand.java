package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;
import lib.Control.SOTA_Xboxcontroller;

public class BasicIntakeCommand extends CommandBase{
    private Intake mIntake;
    private DoubleSupplier mPower;

    public BasicIntakeCommand(Intake intake, DoubleSupplier power) {
        this.mIntake = intake; 
        this.mPower = power;
        addRequirements(mIntake);
    }

    @Override
    public void execute() {
        double speed = mPower.getAsDouble() * 0.5;

        mIntake.set(speed);
    }

    public void setPower(double power) {
        mPower = () -> power;
    }

}
