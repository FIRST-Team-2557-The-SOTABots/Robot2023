// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Config;

import static frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ShiftingSwerveDriveConfig {
    
    private double wheelBase;
    private double trackWidth;
    
    private double maxWheelSpeed;
    private double maxAngularVelocity;
    
    private double getWheelBase() {
        return wheelBase * METERS_PER_INCH;
    }
    
    private double getTrackWidth() {
        return trackWidth * METERS_PER_INCH;
    }

    /**
     * Order Front right, Front left, Back Left, Back Right
     */
    public Translation2d[] getModuleTranslations() {
        Translation2d[] moduleTranslations = {
            new Translation2d(getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(getWheelBase() / 2, getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, getTrackWidth() / 2)
        };
        return moduleTranslations;
    }

    public double getMaxWheelSpeed() {
        return maxWheelSpeed;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

}
