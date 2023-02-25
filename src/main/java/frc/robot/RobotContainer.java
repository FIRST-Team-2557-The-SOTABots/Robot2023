// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModuleI2;
import frc.robot.Util.Configs.MotorControllerConfig;
import frc.robot.Util.Configs.ShiftingSwerveModuleConfig;
import frc.robot.Util.Controllers.CompositeMotor;
import frc.robot.Util.Controllers.FalconDelegate;
import frc.robot.Util.Controllers.SparkMaxDelegate;
import frc.robot.Util.Interfaces.IntakeInterface;
import frc.robot.Util.Interfaces.SOTAMotorController;
import frc.robot.Util.UtilityClasses.ConfigUtils;

public class RobotContainer {
  // private final ArmInterface arm;
  private final IntakeInterface intake;
  private final ConfigUtils configUtils;
  private final CommandXboxController dController;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new CommandXboxController(0);

    SOTAMotorController armExtendMotor = initSparkMaxDelegate("src/main/java/res/armExtendMotor.json");
    

    intake = new Intake(armExtendMotor);

    configureBindings();
  }

  private void configureBindings() {
    dController.a().whileTrue(new RunCommand( () -> {
      intake.intake();
    }, intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  public FalconDelegate initFalconDelegate(String resourceId){
    try{
        MotorControllerConfig config = configUtils.readFromClassPath(MotorControllerConfig.class, resourceId);
        WPI_TalonFX motor = new WPI_TalonFX(config.getPort());
        motor.setInverted(config.getInverted());
        return new FalconDelegate(motor);
    } catch(IOException e) {
      throw new RuntimeException("Error Initializing Talon", e);
    }
  }


  public SparkMaxDelegate initSparkMaxDelegate(String resourceId){
    try{
      MotorControllerConfig config = configUtils.readFromClassPath(MotorControllerConfig.class, resourceId);
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
      CANSparkMax motor = new CANSparkMax(config.getPort(), motorType);
      motor.setInverted(config.getInverted());
      return new SparkMaxDelegate(motor);
    } catch(IOException e){
      throw new RuntimeException("Error Initialzing SparkMax", e);
    }
  }

  public ShiftingSwerveModuleI2 intiSwerveModule(String speedConfig, String angleConfig, int encoderPort, String ModuleConfig){
    SOTAMotorController speedMotor = initFalconDelegate(speedConfig);
    SOTAMotorController angleMotor = initSparkMaxDelegate(angleConfig);
    AnalogInput encoder = new AnalogInput(encoderPort);
    SOTAMotorController angleComposite = new CompositeMotor(angleMotor, encoder);

    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, ModuleConfig);
      return new ShiftingSwerveModuleI2(speedMotor, angleComposite, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
    
  }
}
