// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModuleI2;
import frc.robot.Util.Configs.DoubleSolenoidConfig;
import frc.robot.Util.Configs.MotorControllerConfig;
import frc.robot.Util.Configs.ShiftingSwerveDriveConfig;
import frc.robot.Util.Configs.ShiftingSwerveModuleConfig;
import frc.robot.Util.Controllers.CompositeMotor;
import frc.robot.Util.Controllers.DoubleSolenoidShifter;
import frc.robot.Util.Controllers.FalconDelegate;
import frc.robot.Util.Controllers.SparkMaxDelegate;
import frc.robot.Util.Interfaces.GearShifter;
import frc.robot.Util.Interfaces.IntakeInterface;
import frc.robot.Util.Interfaces.NavX;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;
import frc.robot.Util.UtilityClasses.ConfigUtils;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;
  private final CommandXboxController dController;
  private ShiftingSwerveDrive mSwerveDrive;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new CommandXboxController(0);


    ShiftingSwerveModuleI2[] swerveModules = {
      intiSwerveModule("src/main/java/frc/resources/Swerve/BackLeft/SpeedFalcon.json",
        "src/main/java/frc/resources/Swerve/BackLeft/AngleSparkMax.json",
       1, 
       "src/main/java/frc/resources/Swerve/BackLeft/ShiftingSwerveModule.json"),
       intiSwerveModule("src/main/java/frc/resources/Swerve/BackRight/SpeedFalcon.json",
       "src/main/java/frc/resources/Swerve/BackRight/AngleSparkMax.json",
      2, 
      "src/main/java/frc/resources/Swerve/BackRight/ShiftingSwerveModule.json"),
      intiSwerveModule("src/main/java/frc/resources/Swerve/FrontLeft/SpeedFalcon.json",
        "src/main/java/frc/resources/Swerve/FrontLeft/AngleSparkMax.json",
       0, 
       "src/main/java/frc/resources/Swerve/FrontLeft/ShiftingSwerveModule.json"),
       intiSwerveModule("src/main/java/frc/resources/Swerve/FrontRight/SpeedFalcon.json",
       "src/main/java/frc/resources/Swerve/FrontRight/AngleSparkMax.json",
      3, 
      "src/main/java/frc/resources/Swerve/FrontRight/ShiftingSwerveModule.json"),
     
     
    };

    SOTAGyro gyro = new NavX(new AHRS(Port.kMXP));

    try (DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9)) {
      GearShifter shifter = new DoubleSolenoidShifter(solenoid, configUtils.readFromClassPath(DoubleSolenoidConfig.class, 
      "src/main/java/frc/resources/Swerve/DoubleSolenoidSwerveShifter.json"));
      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro, 
      configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "src/main/java/frc/resources/Swerve/ShiftingSwerveDrive.json"));

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    configureBindings();
  }

  

  private void configureBindings() {
    
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
