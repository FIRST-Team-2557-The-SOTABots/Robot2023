package lib.MotorController;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import lib.Encoder.SOTAEncoder;

public interface SOTAMotorController extends MotorController{
    double getTickVelocity();
    SOTAEncoder getEncoder();
    double getMotorCurrent();
    double getMotorTemperature();
    double getPose();
    // void setLimits(MotorLimits motorLimits);
    // double getLowerLimit();
    // double getUpperLimit();
    // boolean atUpperLimit();
    // boolean atLowerLimit();
    // void resetEncoder();
    // boolean getInBrake();
    // void setBrake();
}
