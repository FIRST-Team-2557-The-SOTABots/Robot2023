package lib.Encoder;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogInputEncoder extends AnalogInput implements SOTAEncoder{
    double offset;

    public AnalogInputEncoder(int port){
        super(port);

    }

    public AnalogInputEncoder(int port, double offset){
        super(port);
    }

    @Override
    public double get() {
        return super.getAverageVoltage() - offset;
    }

    @Override
    public double getAbsolutePosition() {
        return super.getAverageVoltage();
    }

    @Override
    public void reset() {
        offset += super.getAverageVoltage();
    }

    @Override
    public void setPositionOffset(double offset) {
        this.offset = offset;
        
    }

    @Override
    public double getPositionOffset() {
        return offset;
    }

    @Override
    public void close() {
        super.close();
        
    }
    
}
