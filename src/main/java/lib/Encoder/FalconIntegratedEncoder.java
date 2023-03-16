// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Encoder;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

/** Add your docs here. */
public class FalconIntegratedEncoder implements SOTAEncoder {
    private static final double kCountsPerRevolution = 2048;
    private TalonFXSensorCollection mEncoder;

    public FalconIntegratedEncoder(TalonFXSensorCollection encoder) {
        mEncoder = encoder;
    }

    public double getPosition() {
        return mEncoder.getIntegratedSensorPosition();
    }
    
    public void setPosition(double newPosition) {
        mEncoder.setIntegratedSensorPosition(newPosition, 0);
    }

    public double getVelocity() {
        return mEncoder.getIntegratedSensorVelocity();
    }

    public double getCountsPerRevolution() {
        return kCountsPerRevolution;
    }

    public void reset() {
        mEncoder.setIntegratedSensorPosition(0.0, 0);
    }
    
}
