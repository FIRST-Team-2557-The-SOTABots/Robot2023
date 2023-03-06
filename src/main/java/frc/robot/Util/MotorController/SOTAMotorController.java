package frc.robot.util.MotorController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SOTAMotorController extends MotorController{
    double getSensorTickVelocity();
    double getTickPosition();
    double getEncoder();
    double getMotorCurrent();
    double getEncoderCountsPerRevolution();
    double getMotorTemperature();
    void setLimits(MotorLimits motorLimits);
    double getLowerLimit();
    double getUpperLimit();
    // void setIdleMode(); TODO: fix
}
