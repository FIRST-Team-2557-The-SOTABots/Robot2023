package frc.robot.Util.Controllers;



import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class CompositeMotor implements SOTAMotorController{
    private SOTAMotorController motor;
    private AnalogInput encoder;
    private MotorLimits motorLimits;
    private Double countsPerRevolution;

    public CompositeMotor(SOTAMotorController motor, AnalogInput encoder){
            this(motor, encoder, null, motor.getEncoderCountsPerRevolution());

    }

    public CompositeMotor(SOTAMotorController motor,AnalogInput encoder, MotorLimits motorLimits){
        this(motor, encoder, motorLimits, null);
    }

    public CompositeMotor(SOTAMotorController motor,AnalogInput encoder, MotorLimits motorLimits, Double encoderCountsPerRevolution){
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
        return motor.getSensorTickVelocity();
    }

    @Override
    public double getTickPosition() {
        return motor.getTickPosition();
    }

    @Override
    public double getEncoder() {
        return encoder.getAverageVoltage();
    }

    @Override
    public double getMotorCurrent() {
        return motor.getMotorCurrent();
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
        this.motorLimits.setLimits(motorLimits);
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
