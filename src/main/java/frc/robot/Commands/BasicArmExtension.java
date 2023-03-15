package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.SuperStructure;
import lib.Control.SOTAXboxcontroller;

public class BasicArmExtension extends CommandBase{
    private Extension mArm;
    private SOTAXboxcontroller mController;

    public BasicArmExtension(Extension mArm, SOTAXboxcontroller mController){
        this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }
    @Override
    public void execute() {
        if(mController.getA()){
            mArm.set(-2);
        } else if(mController.getB()){
            mArm.set(3);
        } else {
            mArm.set(0);
        }

    }
}
