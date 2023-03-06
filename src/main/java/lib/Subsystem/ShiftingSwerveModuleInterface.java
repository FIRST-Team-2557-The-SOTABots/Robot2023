package lib.Subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModuleState;

public interface ShiftingSwerveModuleInterface extends Subsystem{
    void drive(ShiftingSwerveModuleState state);
    double getSpeed(int gear);
    double getAngle();
    Rotation2d getRotation2d();
    double nativeToMetersPerSecond(double encoderVelocity, double gearRatio);
    double nativeToRadians(double encoderCounts);
    double metersPerSecondToNative(double metersPerSecond, double gearRatio);
    double radiansToNative(double radians);
    double getMetersPerCount(double gearRatio);
    SwerveModulePosition getMeasuredPosition();
}
