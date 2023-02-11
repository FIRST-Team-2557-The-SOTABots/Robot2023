// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.MotorController;

/** Add your docs here. */
public interface SotaMotorController {
    public void set(double speed);
    public double get();
    public void setVoltage(double voltage);
    public double getTickPosition();
    public double getTickVelocity();
    public double getMotorTemperature();
    public double getMotorCurrent();
    public double getEncoderCountsPerRevolution();
}

