package frc.robot.util.Configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static frc.robot.Constants.MATH.*;

public class ShiftingSwerveDriveConfig {
    private double wheelBase;
    private double trackWidth;
    
    private double maxWheelSpeed;
    private double maxAngularVelocity;
    
    public double getWheelBase() {
        return wheelBase * METERS_PER_INCH;
    }
    
    public double getTrackWidth() {
        return trackWidth * METERS_PER_INCH;
    }

    /**
     * Order Front right, Front left, Back Left, Back Right
     */
    public Translation2d[] getModuleTranslations() {
        Translation2d[] moduleTranslations = {
            new Translation2d(getWheelBase() / 2, getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(getWheelBase() / 2, -getTrackWidth() / 2)
        };
        return moduleTranslations;
    }
    public SwerveDriveKinematics getKinematics(){
        return new SwerveDriveKinematics(
            new Translation2d(getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, getTrackWidth() / 2),
            new Translation2d(getWheelBase() / 2, getTrackWidth() / 2));
    }

    public double getMaxWheelSpeed() {
        return maxWheelSpeed;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

}
