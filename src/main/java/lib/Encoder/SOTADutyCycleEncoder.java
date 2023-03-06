package lib.Encoder;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SOTADutyCycleEncoder extends DutyCycleEncoder implements SOTAEncoder{
    public SOTADutyCycleEncoder(DutyCycle dutyCycle) {
        super(dutyCycle);
    }

    public SOTADutyCycleEncoder(int channel) {
        super(channel);
    }


    public SOTADutyCycleEncoder(DigitalSource source) {
        super(source);
    }

    @Override
    public double get() {
        return super.get();
    }
    
}
