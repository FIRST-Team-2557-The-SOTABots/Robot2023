// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import lib.Control.SOTA_Xboxcontroller;

public class DefaultDrive extends CommandBase {
  private ShiftingSwerveDrive mSwerveDrive;
  private SOTA_Xboxcontroller mDriveStick;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(ShiftingSwerveDrive swerveDrive, SOTA_Xboxcontroller driveStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerveDrive = swerveDrive;
    mDriveStick = driveStick;
    addRequirements(mSwerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fwd = mDriveStick.getLeftY(); //TODO convert to double suppliers not directly from stick
    double str = mDriveStick.getLeftX();
    double rot = mDriveStick.getRightX();

    // Squares inputs and preserves sign TODO: make controller class that handles this
    fwd = -Math.signum(fwd) * fwd * fwd; // thought i fixed this 
    str = -Math.signum(str) * str * str;
    rot = -Math.signum(rot) * rot * rot;

    shift(
      mDriveStick.getLeftTriggerAxis(), 
      mDriveStick.getRightTriggerAxis()
    );
    // if(mDriveStick.getA()) mSwerveDrive.setFieldCentricActive(true);
    // if(mDriveStick.getB()) mSwerveDrive.setFieldCentricActive(false); 
    if(mDriveStick.getStart()) mSwerveDrive.resetGyro();
    if(mDriveStick.getY()){//Rotate forward

      double error = (mSwerveDrive.getRotation2d().getRadians() % (Math.PI * 2));
      error = error > Math.PI ? Math.PI - error : error;
      rot = -(Math.abs(error) < 0.1 ? 0 : error) * 0.5; //TODO set 4 to angle kpin config
      // rot = rot < 0.01 ? 0 : rot;
    }
    if(mDriveStick.getX()){//rotate to single substation
      Alliance side = DriverStation.getAlliance();
      if(side == Alliance.Blue){
        double error = (mSwerveDrive.getRotation2d().getRadians() % (Math.PI * 2)) - Math.PI / 2;
        error = error > Math.PI ? Math.PI - error : error;
        rot = -(Math.abs(error) < 0.1 ? 0 : error) * 0.5;
      } else {
        double error = (mSwerveDrive.getRotation2d().getRadians() % (Math.PI * 2)) - Math.PI * 1.5;
        error = error > Math.PI ? Math.PI - error : error;
        rot = -(Math.abs(error) < 0.1 ? 0 : error) * 0.5;
      }

    }
    if(mDriveStick.getA()){//rotate backward
      double error = Math.PI + (mSwerveDrive.getRotation2d().getRadians() % (Math.PI * 2 ));
      rot = -(Math.abs(error) < 0.1 ? 0 : error) * 0.5;
    }



    drive(fwd, str, rot, mSwerveDrive.getRotation2d(), new Translation2d());
  }
  

  protected void drive(double fwd, double str, double rot, Rotation2d angle, Translation2d pointOfRotation) {
    mSwerveDrive.drive(fwd, str, rot, angle, pointOfRotation);
  }

  protected void shift(double lo, double hi) {
    if (lo != 0.0) {
      mSwerveDrive.shift(0);
      mSwerveDrive.setAutoShifting(false);
    }
    if (hi != 0.0){
      mSwerveDrive.shift(1);
      mSwerveDrive.setAutoShifting(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
