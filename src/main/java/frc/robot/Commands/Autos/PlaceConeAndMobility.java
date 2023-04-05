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
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class PlaceConeAndMobility extends ParallelCommandGroup {
  private static double kOuttakeTimeout = 0.5;
  private static double kMobilityTimeout = 5;
  private static double kLineupAuto = 2.0;
  private static double kStrTime = 0.5;
  /** Creates a new OnePieceMobilityAutoBalence. */
  public PlaceConeAndMobility(
    ExtensionPID extensionPID,
    RotationPID rotationPID,
    Intake intake,
    ShiftingSwerveDrive swerveDrive
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      extensionPID,
      rotationPID,
      new SequentialCommandGroup(
        new InstantCommand(
          () -> {
            extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
            rotationPID.setSetpoint(RotationSetpoint.HIGH); // TODO:Change
            intake.set(0.3);
          }, intake
        ),
        new WaitUntilCommand(rotationPID::atSetpoint),
        new WaitUntilCommand(extensionPID::atSetpoint),
        new RunCommand(
          () -> {
            intake.set(-0.3);
          }, intake
        ).withTimeout(kOuttakeTimeout),
        new InstantCommand(
          () -> {
            extensionPID.setSetpoint(ExtensionSetpoint.RESET);
            rotationPID.setSetpoint(RotationSetpoint.RESET);
            intake.stop();
          }, intake
        ),
        new WaitUntilCommand(rotationPID::atSetpoint),
        new WaitUntilCommand(extensionPID::atSetpoint),
        new RunCommand(
          () ->
            swerveDrive.drive(
            new ChassisSpeeds(-1,0,0)
            ), swerveDrive
        ).withTimeout(kMobilityTimeout),
        new RunCommand(
          () -> {
            swerveDrive.drive(
              new ChassisSpeeds(0,0,0)
            );  
          }
        ).withTimeout(kLineupAuto)
      )
    );
  }
}
