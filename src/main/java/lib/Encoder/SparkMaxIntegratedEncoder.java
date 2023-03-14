// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Encoder;

import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SparkMaxIntegratedEncoder implements SOTAEncoder {
    
    private final RelativeEncoder mEncoder;

    public SparkMaxIntegratedEncoder(RelativeEncoder encoder) {
        this.mEncoder = encoder;
    }

    public double getPosition() {
        return mEncoder.getPosition();
    }

    public void setPosition(double newPosition) {
        mEncoder.setPosition(newPosition);    
    }

    public double getVelocity() {
        return mEncoder.getVelocity();
    }

    public double getCountsPerRevolution() {
        return mEncoder.getCountsPerRevolution();
    }

    public void reset() {
        mEncoder.setPosition(0.0);
    }
}
