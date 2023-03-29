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
        FLOOR(49),
        FLOORCONE(55),
        FLOORCONEKNOCK(69),
        SCORE(116),
        SUBSTATION(135),
        SINGLE(98);
    
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
    private double kP;
    private double kPG; // proportional gain against gravity
    private DoubleSupplier kMinAngle;
    private DoubleSupplier kMaxAngle;
    private DoubleSupplier kExtensionlength;
    private double kRotationDelta;
    private double kRotationDeltaProportional;
    private double kMaxExtension;

    public RotationPID(Rotation rotation,
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle,
            SuperStructureConfig config){
        this.mRotation = rotation; 
        this.setpoint = config.getRotationInitSetpoint();
        this.pidController = config.getRotationPIDController();
        this.kP = config.getRotationKP();
        this.kPG = config.getRotationKPG();
        this.kExtensionlength = extensionLength; 
        this.kMinAngle = minAngle; 
        this.kMaxAngle = maxAngle; 
        this.kRotationDelta = config.getRotationDelta();
        this.kRotationDeltaProportional = config.getrotationDeltaProportional();
        this.kMaxExtension = config.getMaxExtension();
        addRequirements(rotation);
    }

    public void setSetpoint(RotationSetpoint newSetpoint) {
        setpoint = newSetpoint.angle;
    }
    

    @Override
    public void execute() {
        // SmartDashboard.getNumber("set p", 0));
        double adjustedSetpoint = MathUtil.clamp(setpoint, kMinAngle.getAsDouble(), kMaxAngle.getAsDouble());

        pidController.setP(kPG - ((kP * kExtensionlength.getAsDouble()) / kMaxExtension));

        double output = Math.sin(mRotation.getRotationRadians()) * (kRotationDelta + (kRotationDeltaProportional * kExtensionlength.getAsDouble() / kMaxExtension)) 
        + pidController.calculate(mRotation.getRotationDegrees(), adjustedSetpoint);

        SmartDashboard.putNumber("output", output);
        // SmartDashboard.putNumber("kPG", kPG);
        // SmartDashboard.putNumber("max extension", kMaxExtension);

        mRotation.set(output);

        SmartDashboard.putNumber("Delta", kRotationDeltaProportional);

        SmartDashboard.putNumber("Angle Output", output);
        // SmartDashboard.putNumber("MinAngle", minAngle.getAsDouble());
        // SmartDashboard.putNumber("maxAngle", maxAngle.getAsDouble());
        SmartDashboard.putNumber("Rotation goal", setpoint);

    }
    
}
