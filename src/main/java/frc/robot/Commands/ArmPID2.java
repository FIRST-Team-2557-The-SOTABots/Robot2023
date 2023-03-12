package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Util.Controllers.SOTAXboxcontroller;

public class ArmPID2 extends CommandBase{
    private Rotation mArm;
    private double setpoint;
    private PIDController pidController;
    private SOTAXboxcontroller controller;
    private DoubleSupplier minAngle;
    private DoubleSupplier maxAngle;

    public ArmPID2(Rotation mArm,
            PIDController pidController,
            double setpoint,
            SOTAXboxcontroller controller,
            DoubleSupplier minAngle,
            DoubleSupplier maxAngle){
        this.mArm = mArm; this.setpoint = setpoint; this.pidController = pidController; this.controller = controller; this.minAngle = minAngle; this.maxAngle = maxAngle;
        addRequirements(mArm);
    }
    

    @Override
    public void execute() {
        
        // if(controller.getA()) setpoint = 60;
        // if(controller.getB()) setpoint = 150;
        double setSetpoint = SmartDashboard.getNumber("rotation setpoint", 0);
        setpoint = setSetpoint;
        
        setpoint = MathUtil.clamp(setpoint, minAngle.getAsDouble(), maxAngle.getAsDouble());

        pidController.setSetpoint(setpoint);
        double output = pidController.calculate(mArm.getRotationDegrees());
        if(mArm.getRotatorEncoder() > 0.587 && mArm.getRotatorEncoder() < 0.701){
            output = pidController.getSetpoint() > 90 ? 3 : -3;
        }
        mArm.set(output);

        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
    }
    
}
