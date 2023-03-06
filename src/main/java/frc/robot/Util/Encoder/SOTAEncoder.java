package frc.robot.util.Encoder;

public interface SOTAEncoder {
    public double get();
    public double getAbsolutePosition();
    public void reset();
    public void setPositionOffset(double offset);
    public double getPositionOffset();
    public void close();
}
