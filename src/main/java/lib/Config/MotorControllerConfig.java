// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Config;

/** Add your docs here. */
public class MotorControllerConfig {

    private boolean isInverted;
    private String idleMode;
    private String motorType;
    private int encoderCountsPerRevolution;

    //TODO: Temp please remove once json problem is fixed
    public MotorControllerConfig(boolean isInverted, String idleMode, String motorType, int encoderCountsPerRevolution) {
        this.isInverted = isInverted;
        this.encoderCountsPerRevolution = encoderCountsPerRevolution;
        this.idleMode = idleMode;
        this.motorType = motorType;
        
    }

    public boolean getInverted() {
        return isInverted;
    }

    public String getIdleMode() {
        return idleMode;
    }

    public String getMotorType() {
        return motorType;
    }

    public int getEncoderCountsPerRevolution() {
        return encoderCountsPerRevolution;
    }

}
