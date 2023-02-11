// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import lib.Config.MotorControllerConfig;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class MotorControllerTest {
    MotorControllerConfig config;

    @BeforeEach
    void setup() throws JsonParseException, JsonMappingException, IOException {
        ObjectMapper om = new ObjectMapper();
        om.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);  
        config = om.readValue(MotorControllerTest.class.getClassLoader().getResourceAsStream("FalconTest.json"), MotorControllerConfig.class);
    }

    @Test
    void invertedTest() {
        assertTrue(!config.getInverted());
    }
    
    @Test
    void motorTypeTest() {
        assertTrue(config.getMotorType().equals("BRUSHLESS"));
    }

    @Test 
    void idleModeTest() {
        assertTrue(config.getIdleMode().equals( "BRAKE"));
    }
}
