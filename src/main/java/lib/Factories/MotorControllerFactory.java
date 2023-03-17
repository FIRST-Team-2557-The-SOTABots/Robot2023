package lib.Factories;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.Config.EncoderConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.MotorLimitsConfig;
import lib.Encoder.AnalogInputEncoder;
import lib.Encoder.SOTADutyCycleEncoder;
import lib.Encoder.SOTAEncoder;
import lib.MotorController.Falcon;
import lib.MotorController.MotorLimits;
import lib.MotorController.SOTAMotorController;
import lib.MotorController.SparkMaxDelegate;

public class MotorControllerFactory {
    
    public static SOTAMotorController generateFalconDelegate(MotorControllerConfig config){
        WPI_TalonFX motor = new WPI_TalonFX(config.getPort());
        motor.setInverted(config.getIsInverted());
        
        SOTAEncoder encoder =  generateEncoder(config.getEncoderConfig()) ;
        return encoder == null ? new Falcon(motor, generateLimits(config.getMotorLimitsConfig())) : 
        new Falcon(motor, encoder , generateLimits(config.getMotorLimitsConfig()));
    }

    public static SOTAMotorController generateSparkDelegate(MotorControllerConfig config){
        if(config == null) return null;
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
        CANSparkMax sparkMax = new CANSparkMax(config.getPort(), motorType);
        SOTAEncoder encoder = (generateEncoder(config.getEncoderConfig()));
        sparkMax.setInverted(config.getIsInverted());
        if(encoder == null){
            SmartDashboard.putBoolean("Failed to create encoder" +  config.getPort(), true);
        return new SparkMaxDelegate(sparkMax
        , generateLimits(config.getMotorLimitsConfig()));
        }
        return new SparkMaxDelegate(sparkMax, encoder, generateLimits(config.getMotorLimitsConfig()));
    }

    public static MotorLimits generateLimits(MotorLimitsConfig config){
            if(config == null) return new MotorLimits(null, null);

            return new MotorLimits(config.getLowerLimit(), config.getUpperLimit(), config.getFinalLimits());
        
    }

    public static SOTAEncoder generateEncoder(EncoderConfig encoderConfig){
        try{
        if (encoderConfig == null ) return null;
        switch(encoderConfig.getEncoderType()){
            case "ANALOG":
                AnalogInput input = new AnalogInput(encoderConfig.getPort());
                return new AnalogInputEncoder(input, encoderConfig);
            case "DUTYCYCLE":
                return new SOTADutyCycleEncoder(encoderConfig.getPort(), encoderConfig.getEncoderOffset());
        
        }
    } catch(NullPointerException e){}
        return null;
    }
}
