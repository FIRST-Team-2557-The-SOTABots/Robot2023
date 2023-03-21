package lib.MotorController;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import lib.Encoder.SOTA_Encoder;

public interface SOTA_MotorController extends MotorController {
    void set(double speed);
    void setVoltage(double voltage);
    double get();
    void setInverted(boolean isInverted);
    boolean getInverted();
    void setNeutralOperation(NeutralOperation neutralOperation);
    SOTA_Encoder getEncoder();
    double getEncoderVelocity();
    double getEncoderPosition();
    double getNativeEncoderVelocity();
    double getNativeEncoderPosition();
    double getMotorTemperature();
    double getMotorCurrent();
    void setCurrentLimit(int amps);
    MotorLimits getLimits();
    void setLimits(MotorLimits limits);
    // double getLowerLimit();
    // double getUpperLimit();
    // boolean atUpperLimit();
    // boolean atLowerLimit();
    // void resetEncoder();
    // boolean getInBrake();
    // void setBrake();


}

