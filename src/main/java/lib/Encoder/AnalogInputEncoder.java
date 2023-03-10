package lib.Encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import lib.Configs.AbsouleEncoderConfig;

public class AnalogInputEncoder implements SOTAAbsoulteEncoder {
    private final AnalogInput mEncoder;
    private final double kCountsPerRevolution;
    private final double kOffset;

    public AnalogInputEncoder(AnalogInput encoder, AbsouleEncoderConfig config) {
        this.mEncoder = encoder;
        this.kCountsPerRevolution = config.getCountsPerRevolution();
        this.kOffset = config.getCountsPerRevolution();
    }

    public double getPosition() {
        return -1 * MathUtil.inputModulus(getPositionNoOffset() - kOffset, 0, kCountsPerRevolution);
    }

    public double getPositionNoOffset() {
        return mEncoder.getAverageVoltage();
    }

    //TODO: you cant setposition on analog input
    public void setPosition(double newPosition) {
        
    }

    //TODO: perhaps add this but it's not really important rn
    public double getVelocity() {
        return 0;
    }

    public void reset() {
        mEncoder.resetAccumulator();        
    }

    public double getCountsPerRevolution() {
        return kCountsPerRevolution;
    }

    public double getPositionOffset() {
        return kOffset;
    }

}