package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class Rotation extends SubsystemBase{
    private SOTAMotorController motor;
    private SOTAGyro gyro;

    public Rotation(SOTAMotorController motor, SOTAGyro gyro){
        this.gyro = gyro; this.motor = motor;
    }

    public void set(double speed){
        motor.set(speed);
    }

    public double getRotatorEncoder(){
        return motor.getEncoder();
      }

    public double getRoll(){
        if(motor.getEncoder() > 0.653){
          return 180 - gyro.getRoll();
        }   //TODO: put in config
        return gyro.getRoll();
      }
      @Override
      public void periodic() {
          SmartDashboard.putNumber("angle", getRoll());
          SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
      }
}
