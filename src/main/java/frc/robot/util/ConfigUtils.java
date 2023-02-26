package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;

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
        try(InputStream in = Thread.currentThread().getContextClassLoader().getResourceAsStream(classpathLocation)) {
            return objectMapper.readValue(in, clazz);
        }
    }
}