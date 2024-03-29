package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Config.SuperStructureConfig;
import lib.MotorController.SOTA_MotorController;

public class Rotation extends SubsystemBase{
    private SOTA_MotorController motor;
    private double kEncoderAtZero;
    private double kEncoderPerDegree;

    public Rotation(SOTA_MotorController motor, SuperStructureConfig config){
      this.motor = motor;
      kEncoderAtZero = config.getEncoderAtZeroDegrees();
      kEncoderPerDegree = config.getEncoderPerDegree();
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public double getRotatorEncoder() {
        return motor.getEncoderPosition();
    }

    public double getRotationDegrees() {
        return (getRotatorEncoder() - kEncoderAtZero) / kEncoderPerDegree;
      }

    public double getRotationRadians() {
        return ((2*Math.PI) / 360) * getRotationDegrees();  
    }
      @Override
      public void periodic() {
          SmartDashboard.putNumber("Arm Angle", getRotationDegrees());
          SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
          SmartDashboard.putNumber("No Offset", motor.getEncoder().getAbsolutePosition());
        //   SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
        //   SmartDashboard.putNumber("angle from native", motor.getNativeEncoderPose());
          SmartDashboard.putNumber("angle from native", motor.getNativeEncoderPosition());
        //   SmartDashboard.putNumber("Rotation motor limit", motor.getMotorLimits().getUpperLimit());
        SmartDashboard.putNumber("Rotation Current", motor.getMotorCurrent());
      }
}
