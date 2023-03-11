package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Util.Controllers.SOTAXboxcontroller;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class BasicArmExtension extends CommandBase{
    private Extension mArm;
    private SOTAXboxcontroller mController;

    public BasicArmExtension(Extension mArm, SOTAXboxcontroller mController){
        this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }
    @Override
    public void execute() { //TODO: create extension PID
        if(mController.getA()){
            mArm.set(-2);
        } else if(mController.getB()){
            mArm.set(3);
        } else {
            mArm.set(0);
        }
        mArm.set(mController.getLeftTriggerAxis());

    }
}
