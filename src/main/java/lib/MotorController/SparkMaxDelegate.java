package lib.MotorController;

import com.revrobotics.CANSparkMax;

import lib.Encoder.SOTAEncoder;
import lib.Encoder.SparkMaxIntegratedEncoder;

public class SparkMaxDelegate implements SOTAMotorController{
    private final CANSparkMax mMotor;
    private final SOTAEncoder mEncoder;
    private final SOTAEncoder mNativeEncoder;
    private MotorLimits motorLimits;
    public SparkMaxDelegate(CANSparkMax motor){
        this(
            motor, 
            new SparkMaxIntegratedEncoder(motor.getEncoder())
        );
    }

    public SparkMaxDelegate(CANSparkMax motor, MotorLimits limits){
        this(
            motor, 
            new SparkMaxIntegratedEncoder(motor.getEncoder()),
            limits
        );
    }
    
    public SparkMaxDelegate(CANSparkMax motor, SOTAEncoder encoder) {
        this.mMotor = motor;
        this.mEncoder = encoder;
        this.mNativeEncoder = new SparkMaxIntegratedEncoder(mMotor.getEncoder());
        this.motorLimits = new MotorLimits(null, null);
    }
    public SparkMaxDelegate(CANSparkMax motor, SOTAEncoder encoder, MotorLimits limits){
        this.mMotor = motor;
        this.mEncoder = encoder;
        this.mNativeEncoder = new SparkMaxIntegratedEncoder(mMotor.getEncoder());
        this.motorLimits = limits;
    }

    public void set(double speed) {
        if(motorLimits != null){
            
            if(speed < 0){
                if(motorLimits.getLowerLimit() > getPose()) speed = 0;
            }else if(speed > 0){
                if(motorLimits.getUpperLimit() < getPose()) speed = 0;
            }
            
        }
        mMotor.set(speed);          
    }

    public void setVoltage(double voltage) {
            if(motorLimits != null){
                
                if(voltage < 0){
                    if(motorLimits.getLowerLimit() > getPose()) voltage = 0;
                }else if(voltage > 0){
                    if(motorLimits.getUpperLimit() < getPose()) voltage = 0;
                }
                
            }
        
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
        return mEncoder.getVelocity();
    }

    public SOTAEncoder getEncoder() {
        return mEncoder;
    }

    public double getNativeVelocity() {
        return mNativeEncoder.getVelocity();
    }
    public double getNativePosition() {
        return mNativeEncoder.get();
    }

    public SOTAEncoder getNativeEncoder() {
        return mNativeEncoder;
    }

    public double getMotorCurrent() {
        return mMotor.getOutputCurrent();
    }

    public double getMotorTemperature() {
        return mMotor.getMotorTemperature();
    }

    @Override
    public void stopMotor() {
        mMotor.stopMotor();;
        
    }

    @Override
    public double getPose() {
        return mEncoder.get();
    }

    @Override
    public double getNativeEncoderPose() {
        return mNativeEncoder.get();
    }
    @Override
    public MotorLimits getMotorLimits() {
        return motorLimits;
    }

    @Override
    public double getTickPosition() {
        return mEncoder.get();
    }

}