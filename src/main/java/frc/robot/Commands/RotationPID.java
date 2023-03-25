package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Rotation;
import lib.Config.SuperStructureConfig;

public class RotationPID extends CommandBase{

    public enum RotationSetpoint {
        RESET(150),
        SUBSTATION(106.5),
        FLOOR(59),
        SCORE(120);
    
        public double angle;
    
        private RotationSetpoint(double angle) {
            this.angle = angle;
        }
    
    }

    private Rotation mArm;
    private double setpoint;
    private PIDController pidController;
    private DoubleSupplier minAngle;
    private DoubleSupplier maxAngle;
    private DoubleSupplier extensionlength;
    private SuperStructureConfig config;
    // private SOTA_Xboxcontroller controller;

    public RotationPID(Rotation mArm,
            PIDController pidController,
            double setpoint,
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle,
            SuperStructureConfig config){
        this.mArm = mArm; 
        this.setpoint = setpoint;
        this.pidController = pidController;
        // this.controller = controller;
        this.extensionlength = extensionLength; 
        this.minAngle = minAngle; 
        this.maxAngle = maxAngle; 
        this.config = config;
        addRequirements(mArm);
    }

    public void setSetpoint(RotationSetpoint newSetpoint) {
        setpoint = newSetpoint.angle;
    }
    

    @Override
    public void execute() {
        //120 && 245 are for placing
        // if(controller.getA()) setpoint = 120;//110; This is from pickup station

        // if(controller.getB()) setpoint = 237;//245; this is from pickup station
        
        // if(controller.getX()) setpoint = 150; // Retract

        // if(controller.getLeftBumper()) setpoint = 106.5;

        // if(controller.getRightBumper()) setpoint = 250;

        // if(controller.getBack()) setpoint = 59;

        // if(controller.getStart()) setpoint = 293;
        
        setpoint = MathUtil.clamp(setpoint, minAngle.getAsDouble(), maxAngle.getAsDouble());

        pidController.setSetpoint(setpoint);

        pidController.setP(0.05 - ((0.03 * extensionlength.getAsDouble()) / 32));
        // SmartDashboard.getNumber("set p", 0));
        double output = Math.sin(mArm.getRotationRadians()) * (config.getRotationDelta() + (config.getRotationDeltaPorportional() * extensionlength.getAsDouble() / 32)) 
        + pidController.calculate(mArm.getRotationDegrees());

        mArm.set(output);

        // SmartDashboard.putNumber("Angle Output", output);
        // SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        // SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);

    }
    
}
