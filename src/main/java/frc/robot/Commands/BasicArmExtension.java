package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Extension;
import lib.Control.SOTA_Xboxcontroller;

public class BasicArmExtension extends CommandBase{
    private Extension mArm;
    private SOTA_Xboxcontroller mController;

    public BasicArmExtension(Extension mArm, SOTA_Xboxcontroller mController){
        this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }
    @Override
    public void execute() {
        if(mController.getA()){
            mArm.setVoltage(-2);
        } else if(mController.getB()){
            mArm.setVoltage(3);
        } else {
            mArm.setVoltage(0);
        }

    }
}
