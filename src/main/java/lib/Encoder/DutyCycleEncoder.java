package lib.Encoder;

import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleEncoder implements SOTAAbsoulteEncoder {
    private final DutyCycle mEncoder;

    public DutyCycleEncoder(DutyCycle encoder) {
        this.mEncoder = encoder;
    }

    //TODO: jon code this 
    public double getPosition() {
        // TODO Auto-generated method stub
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
    
}
