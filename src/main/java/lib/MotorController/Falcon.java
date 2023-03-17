package lib.MotorController;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import lib.Config.MotorControllerConfig;
import lib.Encoder.FalconIntegratedEncoder;
import lib.Encoder.SOTAEncoder;

public class Falcon implements SOTAMotorController {
    private final WPI_TalonFX mMotor;
    private final SOTAEncoder mEncoder;
    private final SOTAEncoder mNativeEncoder;
    private MotorLimits motorLimits;

    public Falcon(WPI_TalonFX motor, MotorControllerConfig config){
        this(
            motor, 
            new FalconIntegratedEncoder(motor.getSensorCollection())
        );
    }

    public Falcon(WPI_TalonFX motor, SOTAEncoder encoder ) {
        this(motor, encoder, new MotorLimits(null, null));
    }

    public Falcon(WPI_TalonFX motor, MotorLimits limits){
        this(motor,  new FalconIntegratedEncoder(motor.getSensorCollection()), limits);
    }

    public Falcon(WPI_TalonFX motor, SOTAEncoder encoder, MotorLimits limits){
        this.mMotor = motor;
        this.mEncoder = encoder;
        this.mNativeEncoder = new FalconIntegratedEncoder(mMotor.getSensorCollection());
        mMotor.setNeutralMode(NeutralMode.Coast); //TODO: fix
    }

    public void set(double speed) {
        // if(motorLimits != null){
            
            // if(speed < 0){
            //     if(motorLimits.getLowerLimit() > getPose()) speed = 0;
            // }else if(speed > 0){
            //     if(motorLimits.getUpperLimit() < getPose()) speed = 0;
            // }
            
        // }
        mMotor.setVoltage(speed);        
    }

    public void setVoltage(double voltage) {
        // if(motorLimits != null){
            
        //     if(voltage < 0){
        //         if(motorLimits.getLowerLimit() > getPose()) voltage = 0;
        //     }else if(voltage > 0){
        //         if(motorLimits.getUpperLimit() < getPose()) voltage = 0;
        //     }
            
        // }
        mMotor.setVoltage(voltage);
    }


    public double get() {
        return mMotor.getMotorOutputPercent();
    }

    public void setInverted(boolean isInverted) {
        mMotor.setInverted(isInverted);        
    }

    public boolean getInverted() {
        return mMotor.getInverted();
    }

    public void disable() {
        mMotor.neutralOutput();        
    }

    public double getTickVelocity() {
        return mMotor.getSelectedSensorVelocity();
    }

    public double getTickPosition() {
        return mEncoder.get();
    }

    public SOTAEncoder getEncoder() {
        return mEncoder;
    }

    public double getNativeVelocity() {
        return mMotor.getSelectedSensorVelocity();
    }

    public double getNativePosition() {
        return mNativeEncoder.get();
    }

    public SOTAEncoder getNativeEncoder() {
        return mNativeEncoder;
    }

    public double getMotorCurrent() {
        return mMotor.getSupplyCurrent();
    }

    public double getMotorTemperature() {
        return mMotor.getTemperature();
    }

    @Override
    public void stopMotor() {
        mMotor.DestroyObject();
        
    }

    @Override
    public double getPose() {
        // TODO Auto-generated method stub
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

}
