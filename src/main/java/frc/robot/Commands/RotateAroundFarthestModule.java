// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class RotateAroundFarthestModule extends DefaultDrive{
  private ShiftingSwerveDrive mSwerveDrive;
  private Translation2d[] mModuleTranslations;
  private Translation2d mFarthestModule;

  private CommandXboxController mDriveStick;

  /** Creates a new RotateAroundFarthestModule. */
  public RotateAroundFarthestModule(ShiftingSwerveDrive swerveDrive, CommandXboxController driveStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(swerveDrive, driveStick);
    mModuleTranslations = mSwerveDrive.getModuleTranslation();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Arbitrarily large number down field for distance calculation doesnt matter how large it is I just choes a number
    Translation2d downField = new Translation2d(999.0, 0.0);
    mFarthestModule = new Translation2d();
    for (int i = 0; i < mModuleTranslations.length; i++) {
      if (mModuleTranslations[i].getDistance(downField) < mFarthestModule.getDistance(downField)) {
        mFarthestModule = mModuleTranslations[i];
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fwd = mDriveStick.getLeftY();
    double str = mDriveStick.getLeftX();
    double rot = mDriveStick.getRightX();

    super.drive(fwd, str, rot, mSwerveDrive.getRotation2d(), mFarthestModule);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}
