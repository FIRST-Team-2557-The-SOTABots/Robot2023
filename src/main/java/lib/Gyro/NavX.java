// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Gyro;

import javax.print.attribute.standard.MediaSize.NA;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavX implements SOTA_Gyro {
  private final AHRS mNavX;
  private final boolean isInverted;

  /** Creates a new NavX. */
  public NavX(AHRS navX) {
    this.mNavX = navX;
    this.isInverted = false;
  }
  public NavX(AHRS navX, boolean isInverted){
    this.mNavX = navX;
    this.isInverted = isInverted;
  }

  public double getAngle() {
    return isInverted ? -1 *  mNavX.getAngle() : mNavX.getAngle();
  }

  public Rotation2d getRotation2d() {
    return isInverted ? new Rotation2d(-mNavX.getRotation2d().getRadians()) : mNavX.getRotation2d(); //TODO: test to make sure this is correct
  }

  public void setAngle(double radians) {
    mNavX.reset();
    mNavX.setAngleAdjustment(radians);
  }

  public void setAngle(Rotation2d rotation2d) {
    mNavX.reset();
    setAngle(rotation2d.getRadians());    
  }

  public void resetAngle() {
    mNavX.reset();
    mNavX.setAngleAdjustment(0.0);
  }

  public double getPitch() {
    return mNavX.getPitch();
  }

  public double getRoll() {
    return mNavX.getRoll();
  }

  public double getYaw() { 
    return mNavX.getYaw();
  }
  
}
