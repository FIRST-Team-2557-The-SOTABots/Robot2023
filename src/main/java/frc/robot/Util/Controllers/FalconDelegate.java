package frc.robot.Util.Controllers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Util.Interfaces.SOTAMotorController;

public class FalconDelegate implements SOTAMotorController{
    WPI_TalonFX motor;
    private MotorLimits motorLimits;
    private Double encoderCountsPerRevolution;

    public FalconDelegate(WPI_TalonFX motor){
        this(motor, null, null);
    }
    public FalconDelegate(WPI_TalonFX motor, MotorLimits motorLimits){
        this(motor, motorLimits, null);
    }
    public FalconDelegate(WPI_TalonFX motor, MotorLimits motorLimits, Double encoderCountsPerRevolution){
        this.motor = motor; ; this.encoderCountsPerRevolution = encoderCountsPerRevolution; this.motorLimits = motorLimits;
    }

    @Override
    public void set(double speed) {
        if(motorLimits != null){
            try{
            if(speed < 0){
                if(motorLimits.getLowerLimit() > getEncoder()) speed = 0;
            }else if(speed > 0){
                if(motorLimits.getUpperLimit() < getEncoder()) speed = 0;
            }
            } catch(NullPointerException e){

            }
        }
        motor.set(speed);        
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);        
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();        
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();        
    }

    @Override
    public double getSensorTickVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public double getTickPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public double getEncoder() {
        return motor.getSelectedSensorPosition() ;
    }

    @Override
    public double getMotorCurrent() {
        return motor.getSupplyCurrent();
    }

    @Override
    public double getEncoderCountsPerRevolution() {
        return encoderCountsPerRevolution;
    }

    @Override
    public double getMotorTemperature() {
        return motor.getTemperature();
    }

    @Override
    public void setLimits(MotorLimits motorLimits) {
        motorLimits.setLimits(motorLimits);
    }

    @Override
    public double getLowerLimit() {
        
        return motorLimits.getUpperLimit();
    }
    @Override
    public double getUpperLimit() {
        return motorLimits.getUpperLimit();
    }
}
