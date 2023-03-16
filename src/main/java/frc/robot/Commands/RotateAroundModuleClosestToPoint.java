// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class RotateAroundModuleClosestToPoint extends DefaultDrive{
  private ShiftingSwerveDrive mSwerveDrive;
  private Translation2d[] mModuleTranslations;
  private Translation2d mClosestModule;
  private Translation2d mPoint;

  private CommandXboxController mDriveStick;

  /** Creates a new RotateAroundFarthestModule. */
  public RotateAroundModuleClosestToPoint(ShiftingSwerveDrive swerveDrive, CommandXboxController driveStick, Translation2d point) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(swerveDrive, driveStick);
    mModuleTranslations = mSwerveDrive.getModuleTranslation();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mClosestModule = new Translation2d();
    for (int i = 0; i < mModuleTranslations.length; i++) {
      if (mModuleTranslations[i].getDistance(mPoint) < mClosestModule.getDistance(mPoint)) {
        mClosestModule = mModuleTranslations[i];
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fwd = mDriveStick.getLeftY();
    double str = mDriveStick.getLeftX();
    double rot = mDriveStick.getRightX();

    // Squares inputs and preserves sign TODO: make controller class that handles this
    fwd = -Math.signum(fwd) * fwd * fwd;
    str = -Math.signum(str) * str * str;
    rot = -Math.signum(rot) * rot * rot;

    super.drive(fwd, str, rot, mSwerveDrive.getRotation2d(), mClosestModule);
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
