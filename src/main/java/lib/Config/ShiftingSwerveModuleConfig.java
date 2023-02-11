// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Config;

/** Add your docs here. */
public class ShiftingSwerveModuleConfig {
    private double loGearRatio;
    private double hiGearRatio;

    private double angleOffset;
    private double angleEncoderCPR;

    private double wheelCircumference;

    private double speedKP;
    private double speedKI;
    private double speedKD;
    private double speedMaxAccel;
    private double speedMaxVel;

    private double speedKS;
    private double speedKV;

    private double angleKP;
    private double angleKI;
    private double angleKD;
    private double angleMaxAccel;
    private double angleMaxVel;

    private double angleKS;
    private double angleKV;

    private double anglePIDTolerance;


    public double getLoGearRatio() {
        return loGearRatio;
    }

    public double getHiGearRatio() {
        return hiGearRatio;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public double getAngleEncoderCPR() {
        return angleEncoderCPR;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
    }

    public double getSpeedKP() {
        return speedKP;
    }

    public double getSpeedKI() {
        return speedKI;
    }

    public double getSpeedKD() {
        return speedKD;
    }

    public double getSpeedMaxAccel() {
        return speedMaxAccel;
    }

    public double getSpeedMaxVel() {
        return speedMaxVel;
    }

    public double getSpeedKS() {
        return speedKS;
    }

    public double getSpeedKV() {
        return speedKV;
    }

    public double getAngleKP() {
        return angleKP;
    }

    public double getAngleKI() {
        return angleKI;
    }

    public double getAngleKD() {
        return angleKD;
    }

    public double getAngleMaxAccel() {
        return angleMaxAccel;
    }

    public double getAngleMaxVel() {
        return angleMaxVel;
    }

    public double getAngleKS() {
        return angleKS;
    }

    public double getAngleKV() {
        return angleKV;
    }

    public double getAnglePIDTolerance() {
        return anglePIDTolerance;
    }

}
