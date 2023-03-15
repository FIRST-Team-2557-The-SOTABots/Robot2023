package lib.Factories;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
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
        motor.setInverted(config.getInverted());
        return new Falcon(motor, generateEncoder(config.getEncoderConfig()), generaLimits(config.getMotorLimitsConfig()));
    }

    public static SOTAMotorController generateSparkDelegate(MotorControllerConfig config){
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
        return new SparkMaxDelegate(sparkMax, generateEncoder(config.getEncoderConfig()), generaLimits(config.getMotorLimitsConfig()));
    }

    public static MotorLimits generaLimits(MotorLimitsConfig config){
        return new MotorLimits(config.getLowerLimit(), config.getUpperLimit(), config.getFinalLimits());
    }

    public static SOTAEncoder generateEncoder(EncoderConfig encoderConfig){
        switch(encoderConfig.getEncoderType()){
            case "ANALOG":
                AnalogInput input = new AnalogInput(encoderConfig.getPort());
                return new AnalogInputEncoder(input, encoderConfig);
            case "DUTYCYCLE":
                return new SOTADutyCycleEncoder(encoderConfig.getPort());
        
        }
        throw new IllegalArgumentException("Error Initializing Encoder");
    }
}
