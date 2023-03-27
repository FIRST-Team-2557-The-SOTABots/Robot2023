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

    private Rotation mRotation;
    private double setpoint;
    private PIDController pidController;
    private DoubleSupplier minAngle;
    private DoubleSupplier maxAngle;
    private DoubleSupplier extensionlength;
    private SuperStructureConfig config;

    public RotationPID(Rotation mRotation,
            PIDController pidController,
            double setpoint,
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle,
            SuperStructureConfig config){

        this.mRotation = mRotation; 
        this.setpoint = setpoint;
        this.pidController = pidController;
        // this.controller = controller;
        this.extensionlength = extensionLength; 
        this.minAngle = minAngle; 
        this.maxAngle = maxAngle; 
        this.config = config;
        addRequirements(mRotation);
    }

    public void setSetpoint(RotationSetpoint newSetpoint) {
        setpoint = newSetpoint.angle;
    }
    

    @Override
    public void execute() {
        setpoint = MathUtil.clamp(setpoint, minAngle.getAsDouble(), maxAngle.getAsDouble());

        pidController.setSetpoint(setpoint);

        pidController.setP(0.05 - ((0.03 * extensionlength.getAsDouble()) / 32));
        // SmartDashboard.getNumber("set p", 0));
        double output = Math.sin(mRotation.getRotationRadians()) * (config.getRotationalDelta() + (config.getRotationalDeltaProportional() * extensionlength.getAsDouble() / 32)) 
        + pidController.calculate(mRotation.getRotationDegrees());


        mRotation.set(output);

        SmartDashboard.putNumber("Delta", config.getRotationalDeltaProportional());

        SmartDashboard.putNumber("Angle Output", output);
        // SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        // SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);

    }
    
}
