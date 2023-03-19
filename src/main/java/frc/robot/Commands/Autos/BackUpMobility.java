// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import lib.Command.AutoCommand;

public class BackUpMobility extends CommandBase{

  public static final double kTime = 1.5;

  private ShiftingSwerveDrive mSwerveDrive;
  /** Creates a new BackUpMobility. */
  public BackUpMobility(ShiftingSwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerveDrive = swerveDrive;
    addRequirements(mSwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSwerveDrive.resetGyro();
    mSwerveDrive.setFieldCentricActive(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSwerveDrive.drive(-0.5, 0, 0, mSwerveDrive.getRotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerveDrive.drive(0.0, 0.0, 0.0, mSwerveDrive.getRotation2d());
    mSwerveDrive.setFieldCentricActive(true);
  }

}
