package lib.MotorController;

import lib.Encoder.SOTAEncoder;

public interface SOTAMotorController {
    void set(double speed);
    void setVoltage(double voltage);
    double get();
    double getTickVelocity();
    double getTickPosition();
    double getEncoder();
    double getMotorCurrent();
    double getMotorTemperature();
    // void setIdleMode(); TODO: fix
}
