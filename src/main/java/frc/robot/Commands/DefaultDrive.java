// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class DefaultDrive extends CommandBase {
  private ShiftingSwerveDrive mSwerveDrive;
  private CommandXboxController mDriveStick;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(ShiftingSwerveDrive swerveDrive, CommandXboxController driveStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerveDrive = swerveDrive;
    mDriveStick = driveStick;
    addRequirements(mSwerveDrive);
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

    shift(
      mDriveStick.getLeftTriggerAxis(), 
      mDriveStick.getRightTriggerAxis()
    );
    drive(fwd, str, rot, mSwerveDrive.getRotation2d(), new Translation2d());
  }

  protected void drive(double fwd, double str, double rot, Rotation2d angle, Translation2d pointOfRotation) {
    mSwerveDrive.drive(fwd, str, rot, angle, pointOfRotation);
  }

  protected void shift(double lo, double hi) {
    if (lo != 0.0) 
      mSwerveDrive.shift(0);
    if (hi != 0.0)
      mSwerveDrive.shift(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
