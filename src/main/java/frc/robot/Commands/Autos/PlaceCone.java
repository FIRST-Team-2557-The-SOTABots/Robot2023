// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import javax.swing.text.html.MinimalHTMLWriter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.AutoLevel;
import frc.robot.Commands.BasicIntakeCommand;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.ResetExtension;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class PlaceCone extends SequentialCommandGroup {
  private static double kOuttakeTimeout = 0.5;
  private static double kMobilityTimeout = 2.5;
  private static double kLineupAuto = 2.0;
  private static double kStrTime = 0.5;
  /** Creates a new OnePieceMobilityAutoBalence. */
  public PlaceCone(
    ExtensionPID extensionPID,
    RotationPID rotationPID,
    BasicIntakeCommand intakeCommand,
    ResetExtension resetExtension
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands( // TODO: Autos violate DRY coding principle
    resetExtension,
    new ParallelCommandGroup(
      extensionPID,
      rotationPID,
      intakeCommand,
      new SequentialCommandGroup(
        new InstantCommand(
          () -> {
            extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
            rotationPID.setSetpoint(RotationSetpoint.HIGH); // TODO:Change
            intakeCommand.setPower(0.3);
          }
        ),
        new WaitUntilCommand(rotationPID::atSetpoint),
        new WaitUntilCommand(extensionPID::atSetpoint),
        new RunCommand(
          () -> {
            intakeCommand.setPower(-0.3);
          }
        ).withTimeout(kOuttakeTimeout),
        new InstantCommand(
          () -> {
            extensionPID.setSetpoint(ExtensionSetpoint.RESET);
            rotationPID.setSetpoint(RotationSetpoint.RESET);
            intakeCommand.setPower(0.0);
          }
        ),
        new WaitUntilCommand(rotationPID::atSetpoint),
        new WaitUntilCommand(extensionPID::atSetpoint)
      ))
    );
  }
}
