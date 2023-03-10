// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Configs;

/** Add your docs here. */
public class AbsouleEncoderConfig {
    private int port;
    private double encoderOffset;
    private double countsPerRevolution;

    public int getPort() {
        return port;
    }

    public double getCountsPerRevolution() {
        return countsPerRevolution;
    }

    public double getEncoderOffset() {
        return encoderOffset;
    }

}
