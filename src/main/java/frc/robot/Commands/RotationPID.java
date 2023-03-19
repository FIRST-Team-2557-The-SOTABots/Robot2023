package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.SuperStructure;
import lib.Config.SuperStructureConfig;
import lib.Control.SOTAXboxcontroller;

public class RotationPID extends CommandBase{
    private Rotation mArm;
    private double setpoint;
    private PIDController pidController;
    private DoubleSupplier minAngle;
    private DoubleSupplier maxAngle;
    private DoubleSupplier extensionlength;
    private SuperStructureConfig config;
    private SOTAXboxcontroller controller;

    public RotationPID(Rotation mArm,
            PIDController pidController,
            double setpoint,
            SOTAXboxcontroller controller,
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle,
            SuperStructureConfig config){
        this.mArm = mArm; this.setpoint = setpoint; this.pidController = pidController; this.controller = controller;
         this.extensionlength = extensionLength; this.minAngle = minAngle; this.maxAngle = maxAngle; this.config = config;
        addRequirements(mArm);
    }
    

    @Override
    public void execute() {
        //120 && 245 are for placing
        if(controller.getA()) setpoint = 120;//110; This is from pickup station

        if(controller.getB()) setpoint = 237;//245; this is from pickup station
        
        if(controller.getX()) setpoint = 180;

        if(controller.getLeftBumper()) setpoint = 110;

        if(controller.getRightBumper()) setpoint = 250;

        if(controller.getBack()) setpoint = 59;

        if(controller.getStart()) setpoint = 293;
        
        setpoint = MathUtil.clamp(setpoint, minAngle.getAsDouble(), maxAngle.getAsDouble());

        pidController.setSetpoint(setpoint);

        pidController.setP(0.05 - ((0.03 * extensionlength.getAsDouble()) / 32));
        // SmartDashboard.getNumber("set p", 0));
        double output = Math.sin(mArm.getRotationRadians()) * (config.getRotationDelta() + (config.getRotationDeltaPorportional() * extensionlength.getAsDouble() / 32)) 
        + pidController.calculate(mArm.getRotationDegrees());

        mArm.set(output);



        SmartDashboard.putNumber("Angle Output", output);
        // SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        // SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);



    }
    
}
