package lib.Encoder;


import java.time.OffsetTime;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SOTADutyCycleEncoder implements SOTAEncoder{
    private double offset;
    private double distancePerRot = 1;
    private DutyCycleEncoder encoder;
    public SOTADutyCycleEncoder(DutyCycle dutyCycle) {
        encoder = new DutyCycleEncoder(dutyCycle);
    }

    public SOTADutyCycleEncoder(int channel) {
        encoder = new DutyCycleEncoder(channel);
    }


    public SOTADutyCycleEncoder(DigitalSource source) {
        encoder = new DutyCycleEncoder(source);
        
    }
    public SOTADutyCycleEncoder(DutyCycle dutyCycle, double positionOffset) {
        encoder = new DutyCycleEncoder(dutyCycle);
        setPositionOffset(positionOffset);
    }

    public SOTADutyCycleEncoder(int channel, double positionOffset) {
        encoder = new DutyCycleEncoder(channel);
        setPositionOffset(positionOffset);
        
    }
    public SOTADutyCycleEncoder(DigitalSource source, double positionOffset) {
        encoder = new DutyCycleEncoder(source);
        setPositionOffset(positionOffset);

    }


    public SOTADutyCycleEncoder(DigitalSource source, double positionOffset, double distance_per_rot) {
        encoder = new DutyCycleEncoder(source);
        setPositionOffset(positionOffset);
        setDistancePerRotation(distance_per_rot);
        
    }public SOTADutyCycleEncoder(DutyCycle dutyCycle, double positionOffset, double distance_per_rot) {
        encoder = new DutyCycleEncoder(dutyCycle);
        setPositionOffset(positionOffset);
        setDistancePerRotation(distance_per_rot);
    }

    public SOTADutyCycleEncoder(int channel, double positionOffset, double distance_per_rot) {
        encoder = new DutyCycleEncoder(channel);
        setPositionOffset(positionOffset);
        setDistancePerRotation(distance_per_rot);

    }

    public void setDistancePerRotation(double distancePerRot) {
        this.distancePerRot = distancePerRot;
    }

    public void setPositionOffset(double offset){
        this.offset = offset;
    }
    
    @Override
    public double get() {
        return  (getAbsolutePosition() - offset < 0 ? 1 + (getAbsolutePosition() - offset) : getAbsolutePosition() - offset) * distancePerRot;
    }

    @Override
    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public void reset() {
        encoder.reset();
        
    }

    @Override
    public double getPositionOffset() {
        return offset;
    }

    @Override
    public void close() {
        encoder.close();
        
    }
    //TODO: FIX
    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getCountsPerRevolution() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
