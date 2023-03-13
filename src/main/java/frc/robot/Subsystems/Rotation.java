package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Configs.SuperStructureConfig;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class Rotation extends SubsystemBase{
    private SOTAMotorController motor;
    private SOTAGyro gyro;
    private SuperStructureConfig config;

    public Rotation(SOTAMotorController motor, SOTAGyro gyro, SuperStructureConfig config){
        this.gyro = gyro; this.motor = motor; this.config = config;
    }

    public void set(double speed){
        motor.set(speed);
    }

    public double getRotatorEncoder(){
        return motor.getEncoder();
      }

    public double getRotationDegrees(){
        return (getRotatorEncoder()-config.getEncoderAtZeroDegrees())/config.getEncoderPerDegree();
      }

    public double getRotationRadians(){
        return ((2*Math.PI) / 360) * getRotationDegrees();  
    }
      @Override
      public void periodic() {
          SmartDashboard.putNumber("Arm Angle", getRotationDegrees());
          SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
          SmartDashboard.putNumber("angle supplied voltage", motor.get());
      }
}
