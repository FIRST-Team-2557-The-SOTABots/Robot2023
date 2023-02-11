// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.MotorController;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import lib.Config.MotorControllerConfig;

/** Add your docs here. */
public class FalconDelegate implements SotaMotorController {
    WPI_TalonFX mFalcon;
    int kEncoderCountsPerRevolution;

    public FalconDelegate(int canId, MotorControllerConfig config) {
        mFalcon = new WPI_TalonFX(canId);
        mFalcon.setInverted(config.getInverted());
        switch(config.getIdleMode()) {
            case("BRAKE"):
                mFalcon.setNeutralMode(NeutralMode.Brake);
                break;
            case("COAST"):
                mFalcon.setNeutralMode(NeutralMode.Coast);
                break;
            default:
                throw new IllegalArgumentException("Illegal idle mode");

        }
        kEncoderCountsPerRevolution = config.getEncoderCountsPerRevolution();
    }

    public static FalconDelegate generateDefaultFalcon(int canId) throws JsonParseException, JsonMappingException, IOException {        
        ObjectMapper mapper = new ObjectMapper();
        mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
        MotorControllerConfig config = mapper.readValue(FalconDelegate.class.getClassLoader().getResourceAsStream("DefaultFalcon.json"), MotorControllerConfig.class);
        return new FalconDelegate(canId, config);
    }

    @Override
    public void set(double speed) {
        mFalcon.set(speed);        
    }

    @Override
    public double get() {
        return mFalcon.get();
    }

    @Override
    public void setVoltage(double voltage) {
        mFalcon.setVoltage(voltage);    
    }

    @Override
    public double getTickPosition() {
        return mFalcon.getSelectedSensorPosition();
    }

    @Override
    public double getTickVelocity() {
        return mFalcon.getSelectedSensorVelocity();
    }

    @Override
    public double getMotorTemperature() {
        return mFalcon.getTemperature();
    }

    @Override
    public double getMotorCurrent() {
        return mFalcon.getSupplyCurrent();
    }

    @Override
    public double getEncoderCountsPerRevolution() {
        return kEncoderCountsPerRevolution;
    }

}

