package lib.MotorController;

import lib.Encoder.SOTAEncoder;

public interface SOTAMotorController {
    void set(double speed);
    void setVoltage(double voltage);
    double get();
    double getTickVelocity();
    double getTickPosition();
    SOTAEncoder getEncoder();
    double getNativeVelocity();
    double getNativePosition();
    SOTAEncoder getNativeEncoder();
    double getMotorCurrent();
    double getMotorTemperature();
    // void setIdleMode(); TODO: fix
}
