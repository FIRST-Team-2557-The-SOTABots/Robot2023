package lib.Encoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import lib.Config.EncoderConfig;

public class SOTADutyCycleEncoder implements SOTAAbsoulteEncoder {
    private final DutyCycleEncoder mEncoder;
    private final EncoderConfig encoderConfig;

    public SOTADutyCycleEncoder(DutyCycleEncoder encoder, EncoderConfig config) {
        this.mEncoder = encoder; this.encoderConfig = config;
    }

    //TODO: jon code this 
    public double getPosition() {
        return 0;
    }

    public void setPosition(double newPosition) {
        // TODO Auto-generated method stub
        
    }

    public double getVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    public void reset() {
        // TODO Auto-generated method stub
        
    }

    public double getPositionOffset() {
        // TODO Auto-generated method stub
        return 0;
    }

    public double getPositionNoOffset() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getCountsPerRevolution() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getOffset() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
