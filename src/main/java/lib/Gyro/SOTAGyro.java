package lib.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public interface SOTAGyro extends Gyro{
    public double getAngle();
    public Rotation2d getRotation2d();
    public void setAngle(double angle);
    public void setAngle(Rotation2d angle);
    public void resetAngle();
    public double getPitch();
    public double getRoll();
    public double getYaw();
}
