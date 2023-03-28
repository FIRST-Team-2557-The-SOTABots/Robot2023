package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;

public class SwerveUtils {

  /**
   * Gets the module positions from the modules
   * @return An array of the position of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions(ShiftingSwerveModule[] swerveModules, int gear) {
    int moduleNum = swerveModules.length;
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleNum];
    for (int i = 0; i < moduleNum; i++) {
      modulePositions[i] = swerveModules[i].getMeasuredPosition(gear);
    }
    return modulePositions;
  }
}
