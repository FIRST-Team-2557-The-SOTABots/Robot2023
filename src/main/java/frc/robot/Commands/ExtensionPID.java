package frc.robot.Commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Util.Controllers.SOTAXboxcontroller;

public class ExtensionPID extends CommandBase{
    private ProfiledPIDController extendPID;
    private SuperStructure mArm;
    private SOTAXboxcontroller mController;

    public ExtensionPID(ProfiledPIDController PID, SuperStructure mArm, SOTAXboxcontroller mController){
        this.extendPID = PID; this.mArm = mArm; this.mController = mController;
        addRequirements(mArm);
    }

    @Override
    public void execute() {
        if(mController.getA()) extendPID.reset(500.0, 0.0);
        if(mController.getB()) extendPID.reset(0.0,0.0);
        double output = extendPID.calculate(mArm.getExtension());
        SmartDashboard.putNumber("extensionSpeed", output);
        mArm.setExtensionSpeed(output);
    }
}
