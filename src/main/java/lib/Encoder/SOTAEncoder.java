package lib.Encoder;

public interface SOTAEncoder {
    public double get();
    public double getAbsolutePosition();
    public void reset();
    public void setPositionOffset(double offset);
    public double getPositionOffset();
    public void close();
    public double getVelocity();
    public double getCountsPerRevolution();
}
