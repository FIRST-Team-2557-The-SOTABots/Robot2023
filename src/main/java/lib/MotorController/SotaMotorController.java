package lib.MotorController;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import lib.Encoder.SOTAEncoder;

public interface SOTAMotorController extends MotorController{
    double getTickVelocity();
    SOTAEncoder getEncoder();
    double getMotorCurrent();
    void setCurrentLimit(int amps);
    double getMotorTemperature();
    double getPose();
    double getNativeEncoderPose();
    // void setLimits(MotorLimits motorLimits);
    // double getLowerLimit();
    // double getUpperLimit();
    MotorLimits getMotorLimits();
    // boolean atUpperLimit();
    // boolean atLowerLimit();
    // void resetEncoder();
    // boolean getInBrake();
    // void setBrake();
    void set(double speed);
    void setVoltage(double voltage);
    double get();
    double getTickPosition();
    double getNativeVelocity();
    double getNativePosition();
    SOTAEncoder getNativeEncoder();

}

