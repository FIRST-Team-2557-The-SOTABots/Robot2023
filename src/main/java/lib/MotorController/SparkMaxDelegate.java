// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.MotorController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import lib.Config.MotorControllerConfig;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class SparkMaxDelegate implements SotaMotorController {
    CANSparkMax mSparkMax;
    int kEncoderCountsPerRevolution;

    public SparkMaxDelegate(int canId) {
        this(canId, new MotorControllerConfig());
    }

    public SparkMaxDelegate(int canId, MotorControllerConfig config) {
        MotorType motorType;
        switch(config.getMotorType()) {
            case("BRUSHLESS"):
                motorType = MotorType.kBrushless;
                break;
            case("BRUSHED"):
                motorType = MotorType.kBrushed;
                break;
            default:
                throw new IllegalArgumentException("Illegal motor type");
        }
        mSparkMax = new CANSparkMax(
            canId, 
            motorType
        );
        mSparkMax.setInverted(config.getInverted());
        switch(config.getIdleMode()) {
            case("BRAKE"):
                mSparkMax.setIdleMode(IdleMode.kBrake);
                break;
            case("COAST"):
                mSparkMax.setIdleMode(IdleMode.kCoast);
                break;
            default:
                throw new IllegalArgumentException("Illegal idle mode");
        }

    }

    @Override
    public void set(double speed) {
        mSparkMax.set(speed);
    }

    @Override
    public double get() {
        return mSparkMax.get();
    }

    @Override
    public void setVoltage(double voltage) {
        mSparkMax.setVoltage(voltage);        
    }

    @Override
    public double getTickPosition() {
        return mSparkMax.getEncoder().getPosition();
    }

    @Override
    public double getTickVelocity() {
        return mSparkMax.getEncoder().getVelocity();
    }

    @Override
    public double getMotorTemperature() {
        return mSparkMax.getMotorTemperature();
    }

    @Override
    public double getMotorCurrent() {
        return mSparkMax.getOutputCurrent();
    }
    
    @Override
    public double getEncoderCountsPerRevolution() {
        return kEncoderCountsPerRevolution;
    }

}
