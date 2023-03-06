package frc.robot.util.Subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModuleState;

public interface ShiftingSwerveModuleInterface extends Subsystem{
    void drive(ShiftingSwerveModuleState state);
    double getSpeed(int gear);//TODO: make it so it doesnt require the current gear
    double getAngle();
    Rotation2d getRotation2d();
    double nativeToMetersPerSecond(double encoderVelocity, double gearRatio);
    double nativeToRadians(double encoderCounts);
    double metersPerSecondToNative(double metersPerSecond, double gearRatio);//TODO: make it so it requires gear not ratio
    double radiansToNative(double radians);
    double getMetersPerCount(double gearRatio);//TODO: fix so it doesnt require gear ratio but only gear
    SwerveModulePosition getMeasuredPosition();
}
