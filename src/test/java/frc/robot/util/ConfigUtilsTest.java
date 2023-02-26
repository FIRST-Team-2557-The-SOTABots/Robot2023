// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import lib.Config.ShiftingSwerveDriveConfig;

/** Add your docs here. */
public class ConfigUtilsTest {
    @Test
    public void readsFile() throws Exception{
        ObjectMapper mapper = new ObjectMapper();
        mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
        ConfigUtils configUtil = new ConfigUtils(mapper);

        configUtil.readConfigFromClasspath("Swerve/ShiftingSwerveDrive.json", ShiftingSwerveDriveConfig.class);
    }
}
