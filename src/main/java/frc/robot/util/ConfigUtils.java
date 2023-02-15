package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.RobotContainer;
import lib.Config.ShiftingSwerveDriveConfig;

import java.io.IOException;
import java.io.InputStream;

public class ConfigUtils {

    private ObjectMapper objectMapper;

    public ConfigUtils() {
        this(new ObjectMapper());

    }
    public ConfigUtils(ObjectMapper objectMapper) {
        this.objectMapper = objectMapper;
    }

    public <T> T readConfigFromClasspath(String classpathLocation, Class<T> clazz) throws IOException {
        try(InputStream in = ConfigUtils.class.getClassLoader().getResourceAsStream(classpathLocation)) {
            return objectMapper.readValue(in, clazz);
        }
    }
}