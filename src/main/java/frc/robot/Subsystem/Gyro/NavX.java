// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Component.Gyro;

public class NavX extends SubsystemBase implements Gyro{
  private AHRS mNavX;

  /** Creates a new NavX. */
  public NavX(AHRS navX) {
    mNavX = navX;
  }

  @Override
  public double getAngle() {
    return mNavX.getAngle();
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return mNavX.getRotation2d();
  }

  @Override
  public void setAngle(double radians) {
    mNavX.reset();
    mNavX.setAngleAdjustment(radians);
  }

  @Override
  public void setAngle(Rotation2d rotation2d) {
    setAngle(rotation2d.getRadians());    
  }

  @Override
  public void resetAngle() {
    mNavX.reset();
    mNavX.setAngleAdjustment(0.0);
  }

  @Override
  public double getPitch() {
    return mNavX.getPitch();
  }

  @Override
  public double getRoll() {
    return mNavX.getRoll();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
