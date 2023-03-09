package frc.robot.Util.Interfaces;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Util.Controllers.MotorLimits;

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
    boolean atUpperLimit();
    boolean atLowerLimit();
    // void setIdleMode(); TODO: fix
}
