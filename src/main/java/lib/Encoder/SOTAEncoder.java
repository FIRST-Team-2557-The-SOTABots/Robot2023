package lib.Encoder;

public interface SOTAEncoder {
    public double getPosition();
    public void setPosition(double newPosition);
    public double getVelocity();
    public double getCountsPerRevolution();
    public void reset();

}
