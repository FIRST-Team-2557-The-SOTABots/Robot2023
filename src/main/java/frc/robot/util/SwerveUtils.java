package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;

public class SwerveUtils {

    /**
     * Gets the module positions from the modules
     * @return An array of the position of the swerve modules
     */
    public static SwerveModulePosition[] getModulePositions(ShiftingSwerveModule[] swerveModules) {
        int moduleNumber = swerveModules.length;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleNumber];
        for (int i = 0; i < moduleNumber; i++) {
            modulePositions[i] = swerveModules[i].getMeasuredPosition();
        }
        return modulePositions;
    }
}
