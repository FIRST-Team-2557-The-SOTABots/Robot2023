package lib.MotorController;

import com.revrobotics.CANSparkMax;

import lib.Configs.MotorControllerConfig;
import lib.Encoder.SOTAEncoder;
import lib.Encoder.SparkMaxIntegratedEncoder;

public class SparkMax implements SOTAMotorController{
    private final CANSparkMax mMotor;
    private final SOTAEncoder mEncoder;

    public SparkMax(CANSparkMax motor, MotorControllerConfig config){
        this(
            motor, 
            new SparkMaxIntegratedEncoder(motor.getEncoder()),
            config
        );
        setInverted(config.getInverted());
    }
    
    public SparkMax(CANSparkMax motor, SOTAEncoder encoder, MotorControllerConfig config) {
        this.mMotor = motor;
        this.mEncoder = encoder;
    }

    public void set(double speed) {
        mMotor.set(speed);        
    }

    public void setVoltage(double voltage) {
        mMotor.setVoltage(voltage);
    }

    public double get() {
        return mMotor.get();
    }

    public void setInverted(boolean isInverted) {
        mMotor.setInverted(isInverted);        
    }

    public boolean getInverted() {
        return mMotor.getInverted();
    }

    public void disable() {
        mMotor.disable();        
    }

    public double getTickVelocity() {
        return mMotor.getEncoder().getVelocity();
    }

    public double getTickPosition() {
        return mMotor.getEncoder().getPosition();
    }

    public SOTAEncoder getEncoder() {
        return mEncoder;
    }

    public double getMotorCurrent() {
        return mMotor.getOutputCurrent();
    }

    public double getMotorTemperature() {
        return mMotor.getMotorTemperature();
    }


}