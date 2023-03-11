package frc.robot.Commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Util.Controllers.SOTAXboxcontroller;

public class ExtensionPID extends CommandBase{
    private ProfiledPIDController extendPID;
    private Extension mExtension;
    private SOTAXboxcontroller mController;

    public ExtensionPID(ProfiledPIDController PID, Extension mArm, SOTAXboxcontroller mController){
        this.extendPID = PID; this.mExtension = mArm; this.mController = mController;
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        if(mController.getLeftBumper()) extendPID.reset(300.0, 0.0);
        if(mController.getRightBumper()) extendPID.reset(0.0,0.0);
        double output = extendPID.calculate(mExtension.getEncoder());
        SmartDashboard.putNumber("extensionSpeed", output);
        mExtension.set(output);
    }
}
