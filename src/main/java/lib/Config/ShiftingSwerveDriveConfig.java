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
    
    private double getWheelBase() {
        return wheelBase * METERS_PER_INCH;
    }
    
    private double getTrackWidth() {
        return trackWidth * METERS_PER_INCH;
    }

    public Translation2d getFrontRightModulePosition() {
        return new Translation2d(getWheelBase() / 2, -getTrackWidth() / 2);
    }

    public Translation2d getFrontLeftModulePosition() {
        return new Translation2d(getWheelBase() / 2, getTrackWidth() / 2);
    }

    public Translation2d getBackRightModulePosition() {
        return new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2);
    }
    
    public Translation2d getBackLeftModulePosition() {
        return new Translation2d(-getWheelBase() / 2, getTrackWidth() / 2);
    }

    public double getMaxWheelSpeed() {
        return maxWheelSpeed;
    }

}
