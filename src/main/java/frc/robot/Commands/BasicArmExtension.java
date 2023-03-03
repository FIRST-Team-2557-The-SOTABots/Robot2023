package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm;
import frc.robot.Util.Controllers.SOTAXboxcontroller;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class BasicArmExtension extends CommandBase{
    private Arm mArm;
    private SOTAXboxcontroller mController;

    public BasicArmExtension(Arm mArm, SOTAXboxcontroller mController){
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
