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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommand;
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
      intiSwerveModule("Swerve/BackLeft/SpeedFalcon",
        "Swerve/BackLeft/AngleSparkMax",
       1, 
       "Swerve/BackLeft/ShiftingSwerveModule"),
       intiSwerveModule("Swerve/BackRight/SpeedFalcon",
       "Swerve/BackRight/AngleSparkMax",
      2, 
      "Swerve/BackRight/ShiftingSwerveModule"),
      intiSwerveModule("Swerve/FrontLeft/SpeedFalcon",
        "Swerve/FrontLeft/AngleSparkMax",
       0, 
       "Swerve/FrontLeft/ShiftingSwerveModule"),
       intiSwerveModule("Swerve/FrontRight/SpeedFalcon",
       "Swerve/FrontRight/AngleSparkMax",

      3, 
      "Swerve/FrontRight/ShiftingSwerveModule"),
     
     
    };

    SOTAGyro gyro = new NavX(new AHRS(Port.kMXP));

    try{
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
      GearShifter shifter = new DoubleSolenoidShifter(solenoid, 
        configUtils.readFromClassPath(DoubleSolenoidConfig.class, 
        "Swerve/DoubleSolenoidSwerveShifter"));
      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro, 
      configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "Swerve/ShiftingSwerveDrive"));

    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Faild to create swerveDrive", e);
    }
    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    mSwerveDrive.setDefaultCommand(new DriveCommand(mSwerveDrive, dController));
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
        return new FalconDelegate(motor, null, config.getCountsPerRevolution());
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
      return new SparkMaxDelegate(motor, null, config.getCountsPerRevolution());
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
