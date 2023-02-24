// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Vision.Limelight;

public class UpdateDriveOdometryLimelight extends CommandBase {
  private ShiftingSwerveDrive mSwerveDrive;
  private Limelight mLimelight;
  /** Creates a new UpdateDriveOdometryLimelight. */
  public UpdateDriveOdometryLimelight(ShiftingSwerveDrive swerveDrive, Limelight limelight) {
    mSwerveDrive = swerveDrive;
    mLimelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerveDrive, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
