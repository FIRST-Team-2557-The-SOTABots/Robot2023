package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SuperStructure;
import lib.Control.SOTAXboxcontroller;

public class BasicArmExtension extends CommandBase{
    private SuperStructure mArm;
    private SOTAXboxcontroller mController;

    public BasicArmExtension(SuperStructure mArm, SOTAXboxcontroller mController){
        this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }
    @Override
    public void execute() {
        if(mController.getA()){
            mArm.setExtensionSpeed(-2);
        } else if(mController.getB()){
            mArm.setExtensionSpeed(3);
        } else {
            mArm.setExtensionSpeed(0);
        }

    }
}
