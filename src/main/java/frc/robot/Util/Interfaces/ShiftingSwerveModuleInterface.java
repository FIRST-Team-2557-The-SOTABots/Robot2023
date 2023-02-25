package frc.robot.Util.Interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShiftingSwerveModuleInterface {
    void drive(ShiftingSwerveModuleInterface state);
    double getSpeed();
    double getAngle();
    Rotation2d geRotation2d();
    double nativeToMetersPerSecond(double encoderCounts);
    double nativeToRadians(double encoderCounts);
    double metersPerSecondToNative(double mps);
    double radiansToNative(double radians);
    double getMetersPerCount();
}
