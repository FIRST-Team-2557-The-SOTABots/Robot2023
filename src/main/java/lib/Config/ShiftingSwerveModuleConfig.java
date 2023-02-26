// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Config;

import frc.robot.Constants;

/** Add your docs here. */
public class ShiftingSwerveModuleConfig {
    private double loGearRatio;
    private double hiGearRatio;

    private double angleOffset;
    private double angleEncoderCPR;

    private double wheelDiameter;

    private double speedKP;
    private double speedKI;
    private double speedKD;
    private double speedMaxAccel;
    private double speedMaxVel;

    private double speedPIDTolerance;

    private double speedKS;
    private double speedKV;

    private double angleKP;
    private double angleKI;
    private double angleKD;
    private double angleMaxAccel;
    private double angleMaxVel;

    private double anglePIDTolerance;

    private double angleKS;
    private double angleKV;

    public ShiftingSwerveModuleConfig(
        double loGearRatio, 
        double hiGearRatio, 
        double angleOffset, 
        double angleEncoderCPR, 
        double wheelDiameter, 
        double speedKP,
        double speedKI,
        double speedKD,
        double speedPIDTolerance,
        double speedMaxAccel,
        double speedMaxVel,
        double speedKS,
        double speedKV,
        double angleKP,
        double angleKI,
        double angleKD,
        double anglePIDTolerance,
        double angleMaxAccel,
        double angleKS,
        double angleKV) {
        this.loGearRatio = loGearRatio;
        this.hiGearRatio = hiGearRatio;
        this.angleOffset = angleOffset;
        this.angleEncoderCPR = angleEncoderCPR;
        this.wheelDiameter = wheelDiameter;
        this.speedKP = speedKP;
        this.speedKI = speedKI;
        this.speedKD = speedKD;
        this.speedPIDTolerance = speedPIDTolerance;
        this.speedMaxAccel = speedMaxAccel;
        this.speedMaxVel = speedMaxVel;
        this.speedKS = speedKS;
        this.speedKV = speedKV;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.anglePIDTolerance = anglePIDTolerance;
        this.angleMaxAccel = angleMaxAccel;
        this.angleKS = angleKS;
        this.angleKV = angleKV;

    }

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
        return wheelDiameter * Constants.METERS_PER_INCH * Math.PI;
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

    public double getSpeedPIDTolerance() {
        return speedPIDTolerance;
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

    public double getAnglePIDTolerance() {
        return angleMaxAccel * Math.sqrt((angleEncoderCPR / 4) / angleMaxAccel);
    }

    public double getAngleKS() {
        return angleKS;
    }

    public double getAngleKV() {
        return angleKV;
    }

}
