package frc.robot.util.MotorController;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Encoder.SOTAEncoder;

public class CompositeMotor implements SOTAMotorController{
    private SOTAMotorController motor;
    private SOTAEncoder encoder;
    private MotorLimits motorLimits;
    private Double countsPerRevolution;

    public CompositeMotor(SOTAMotorController motor, SOTAEncoder encoder){
            this(motor, encoder, null, motor.getEncoderCountsPerRevolution());

    }

    public CompositeMotor(SOTAMotorController motor,SOTAEncoder encoder, MotorLimits motorLimits){
        this(motor, encoder, motorLimits, null);
    }

    public CompositeMotor(SOTAMotorController motor,SOTAEncoder encoder, MotorLimits motorLimits, Double encoderCountsPerRevolution){
        this.motor = motor; this.motorLimits = motorLimits; this.countsPerRevolution = encoderCountsPerRevolution; this.encoder = encoder;
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
        SmartDashboard.putNumber("armSpeed", speed);
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
        return encoder.get();
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
