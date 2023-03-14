package lib.Configs;

import com.revrobotics.CANSparkMax;

import lib.Encoder.SOTAAbsoulteEncoder;
import lib.Encoder.SOTAEncoder;
import lib.MotorController.MotorLimits;
import lib.MotorController.SOTAMotorController;

public class MotorControllerConfig  {
    private int port;
    private boolean isInverted;
    private String motorType;
    private int countsPerRevolution;
    private String idleMode;
    private EncoderConfig encoder;
    private MotorLimits motorLimits;

    public boolean getInverted() {
        return isInverted;
    }

    public String getIdleMode() {
        return idleMode;
    }

    public String getMotorType() {
        return motorType;
    }

    public double getCountsPerRevolution() {
        return (double)countsPerRevolution;
    }

    public int getPort(){
        return this.port;
    }
    public SOTAEncoder getEncoder(){
        return encoder.getEncoder();
    }

    public MotorLimits getMotorLimits(){
        return motorLimits;
    }
    
}
