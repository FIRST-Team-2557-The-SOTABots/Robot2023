package lib.Config;


public class MotorControllerConfig  {
    private int port;
    private boolean isInverted;

    private String motorType;
    private int countsPerRevolution;
    private String idleMode;
    private EncoderConfig encoderConfig;
    private MotorLimitsConfig motorLimitsConfig;
    private int currentLimit;

    public boolean getIsInverted() {
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
    public EncoderConfig getEncoderConfig(){
        return encoderConfig;
    }

    public MotorLimitsConfig getMotorLimitsConfig(){
        return motorLimitsConfig;
    }

    public int getCurrentLimit() {
        return currentLimit;
    }
    
}
