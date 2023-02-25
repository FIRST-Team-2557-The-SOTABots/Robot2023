package frc.robot.Util.UtilityClasses;

import java.io.File;
import java.io.IOException;


import com.fasterxml.jackson.databind.ObjectMapper;


public class ConfigUtils{
    private ObjectMapper mapper;

    public ConfigUtils(){
        this(new ObjectMapper());
    }

    public ConfigUtils(ObjectMapper mapper){
        this.mapper = mapper;
    }

    public <T> T readFromClassPath(Class<T> clazz, String resource)throws IOException{
            File json = new File(resource);
            return mapper.readValue(json, clazz);
        
    }

}