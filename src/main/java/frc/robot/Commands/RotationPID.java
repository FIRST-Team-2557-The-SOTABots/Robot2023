package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Rotation;
import lib.Control.SOTAXboxcontroller;

public class RotationPID extends CommandBase{
    private Rotation mArm;
    private double setpoint;
    private PIDController pidController;
    private DoubleSupplier minAngle;
    private DoubleSupplier maxAngle;
    private DoubleSupplier extensionlength;

    public RotationPID(Rotation mArm,
            PIDController pidController,
            double setpoint,
            SOTAXboxcontroller controller,
            DoubleSupplier extensionLength){
        this.mArm = mArm; this.setpoint = setpoint; this.pidController = pidController;
         this.extensionlength = extensionLength;
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
        pidController.setP(0.05 - (0.02 * extensionlength.getAsDouble() / 31));

        double output = Math.sin(mArm.getRotationRadians()) * (0.01 + (0.02 * extensionlength.getAsDouble() / 31))//SmartDashboard.getNumber("Test delta", 0) TODO: change to full length 
        + pidController.calculate(mArm.getRotationDegrees());

        // mArm.set(output);

        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);



    }
    
}
