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
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle){
        this.mArm = mArm; this.setpoint = setpoint; this.pidController = pidController;
         this.extensionlength = extensionLength; this.minAngle = minAngle; this.maxAngle = maxAngle;
        addRequirements(mArm);
    }
    

    @Override
    public void execute() {
        
        // if(controller.getA()) setpoint = 60;
        // if(controller.getB()) setpoint = 150;
        double setSetpoint = SmartDashboard.getNumber("rotation setpoint", 180);
        setpoint = setSetpoint;
        
        setpoint = MathUtil.clamp(setpoint, minAngle.getAsDouble(), maxAngle.getAsDouble());

        pidController.setSetpoint(setpoint);

        pidController.setP(//0.05 - ((0.03 * extensionlength.getAsDouble()) / 27));
        SmartDashboard.getNumber("set p", 0));
        double output = Math.sin(mArm.getRotationRadians()) * SmartDashboard.getNumber("Test delta", 0);//(0.01 + (0.02 * extensionlength.getAsDouble() / 27)) 
        // + pidController.calculate(mArm.getRotationDegrees());

        mArm.set(output);

        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);



    }
    
}
