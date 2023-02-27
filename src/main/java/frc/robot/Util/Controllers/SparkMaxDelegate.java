package frc.robot.Util.Controllers;


import com.revrobotics.CANSparkMax;

import frc.robot.Util.Interfaces.SOTAMotorController;

public class SparkMaxDelegate implements SOTAMotorController{
    private CANSparkMax motor;
    private MotorLimits motorLimits;
    private Double countsPerRevolution;

    public SparkMaxDelegate(CANSparkMax motor){
        this(motor, null,  null);
    }
    public SparkMaxDelegate(CANSparkMax motor, MotorLimits motorLimits){
        this(motor, motorLimits, null);
    }
    
    public SparkMaxDelegate(CANSparkMax motor, MotorLimits motorLimits, Double encoderCountsPerRevolution){
        this.motor = motor; this.motorLimits = motorLimits; this.countsPerRevolution = encoderCountsPerRevolution;
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
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getTickPosition() {
        return motor.getEncoder().getPosition();
    }

    
    @Override
    public double getEncoder() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getEncoderCountsPerRevolution() {
        return countsPerRevolution;
    }

    @Override
    public double getMotorTemperature() {
        return motor.getMotorTemperature();
    }

    @Override
    public void setLimits(MotorLimits motorLimits) {
        motorLimits.setLimits(motorLimits);
    }

    @Override
    public double getLowerLimit() {
        return motorLimits.getLowerLimit();
    }
    @Override
    public double getUpperLimit() {
        return motorLimits.getUpperLimit();
    }
}
