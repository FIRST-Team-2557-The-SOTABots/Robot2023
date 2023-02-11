// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Component;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface Gyro {

    public double getAngle();
    public Rotation2d getAngleRotation2d();

    public void setAngle(double radians);
    public void setAngle(Rotation2d rotation2d);

    public void resetAngle();

    public double getPitch();
    public double getRoll();
}
