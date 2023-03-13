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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.ArmPID;
import frc.robot.Commands.ArmPID2;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;
import frc.robot.util.ConfigUtils;
import lib.Configs.DoubleSolenoidConfig;
import lib.Configs.EncoderConfig;
import lib.Configs.MotorControllerConfig;
import lib.Configs.ShiftingSwerveDriveConfig;
import lib.Configs.ShiftingSwerveModuleConfig;
import lib.Control.SOTAXboxcontroller;
import lib.Encoder.AnalogInputEncoder;
import lib.Encoder.SOTAEncoder;
import lib.Gyro.NavX;
import lib.Gyro.Pigeon;
import lib.Gyro.SOTAGyro;
import lib.MotorController.Falcon;
import lib.MotorController.MotorLimits;
import lib.MotorController.SOTAMotorController;
import lib.MotorController.SparkMax;
import lib.Pneumatics.DoubleSolenoidShifter;
import lib.Pneumatics.GearShifter;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;

  private final SOTAGyro gyro;

  private final SOTAXboxcontroller dController;
  private final SOTAXboxcontroller mController;

  private ShiftingSwerveDrive mSwerveDrive;
  private SuperStructure mArm;

  private ArmPID2 mArmPID;
  private DriveCommand mDriveCommand;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new SOTAXboxcontroller(0);
    mController = new SOTAXboxcontroller(1);

    this.gyro = new NavX(new AHRS(Port.kMXP));

    ShiftingSwerveModule[] swerveModules = {
      initSwerveModule(
        "Swerve/FrontLeft/SpeedFalcon",
        "Swerve/FrontLeft/AngleSparkMax",
        "Swerve/FrontLeft/AbsoluteEncoder",
        "Swerve/FrontLeft/ShiftingSwerveModule"
      ),

      initSwerveModule(
        "Swerve/BackLeft/SpeedFalcon",
        "Swerve/BackLeft/AngleSparkMax",
        "Swerve/BackLeft/AbsoluteEncoder",
      "Swerve/BackLeft/ShiftingSwerveModule"
      ),

      initSwerveModule(
        "Swerve/BackRight/SpeedFalcon",
        "Swerve/BackRight/AngleSparkMax",
        "Swerve/BackRight/AbsoluteEncoder",
        "Swerve/BackRight/ShiftingSwerveModule"
      ),
      
      initSwerveModule(
        "Swerve/FrontRight/SpeedFalcon",
        "Swerve/FrontRight/AngleSparkMax",
        "Swerve/FrontLeft/AbsoluteEncoder",
        "Swerve/FrontRight/ShiftingSwerveModule"
      ),
    };

    

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
    // MotorLimits motorLimits = new MotorLimits(0.45, 0.65);
    // SOTAMotorController winchMotor = createSparkMax("SuperStructure/WinchMotor");
    // SOTAMotorController rotatorMotor = createSparkMax("SuperStructure/RotatorMotor");
    // SOTAEncoder rotatorEncoder = new DutyCycleEncoder(1);
    // SOTAMotorController rotatorComposite = new CompositeMotor(rotatorMotor, rotatorEncoder, motorLimits);
    // SOTAGyro armGyro = new Pigeon(4);
    // DigitalInput limitSwitch = new DigitalInput(0);
    // SOTAMotorController leftMotorIntake = createSparkMax("SuperStructure/IntakeMotorLeft");
    // SOTAMotorController rightMotorIntake = createSparkMax("SuperStructure/IntakeMotorRight");
    // SOTAMotorController intakeMotors = new SOTAMotorControllerGroup(rightMotorIntake, leftMotorIntake);
    // this.mArm = new SuperStructure(armGyro, winchMotor, rotatorComposite, limitSwitch, intakeMotors);

    // PIDController armController = new PIDController(0.03,0,0);

    // this.mArmPID = new ArmPID2( mArm, armController, 0, mController);

    this.mDriveCommand = new DriveCommand(mSwerveDrive, dController);


    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    mSwerveDrive.setDefaultCommand(mDriveCommand);
    // mArm.setDefaultCommand(mArmPID);
  }

  private void configureBindings() {
    dController.a().onTrue(new InstantCommand(
      () -> {
        mSwerveDrive.shift();
      }, mSwerveDrive
    ));
    // TODO: add reset gyro
    // dController.b().onTrue(new InstantCommand(
    //   () -> {
    //   }
    // ));A
    dController.x().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(true);
    }));
    dController.y().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(false);
    }));
    
      
  }

  public Command getAutonomousCommand() {
    return new RunCommand(() ->{
      // mSwerveDrive.drive(1, 0, 0, gyro.getRotation2d(), new Translation2d());
    }, mSwerveDrive);
  }


  private Falcon createFalcon(String resourceId){
    try{
        MotorControllerConfig config = configUtils.readFromClassPath(MotorControllerConfig.class, resourceId);
        WPI_TalonFX motor = new WPI_TalonFX(config.getPort());
        return new Falcon(motor, config);
    } catch(IOException e) {
      throw new RuntimeException("Error Initializing Talon", e);
    }
  }

  public SparkMax createSparkMax(String resourceId){
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
      return new SparkMax(motor, config);
    } catch(IOException e){
      throw new RuntimeException("Error Initialzing SparkMax", e);
    }
  }

  public SparkMax createCompositeSparkMax(String motorResource, String encoderResource) {
    try{
      MotorControllerConfig motorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, motorResource);
      EncoderConfig encoderConfig = configUtils.readFromClassPath(EncoderConfig.class, encoderResource);
      MotorType motorType;
      switch(motorConfig.getMotorType()) {
          case("BRUSHLESS"):
              motorType = MotorType.kBrushless;
              break;
          case("BRUSHED"):
              motorType = MotorType.kBrushed;
              break;
          default:
              throw new IllegalArgumentException("Illegal motor type");
      }
      CANSparkMax motor = new CANSparkMax(motorConfig.getPort(), motorType);
      AnalogInputEncoder encoder = new AnalogInputEncoder(new AnalogInput(encoderConfig.getPort()), encoderConfig);
      return new SparkMax(motor, encoder, motorConfig);
    } catch(IOException e){
      throw new RuntimeException("Error Initialzing SparkMax", e);
    }
  } 

  public SOTAEncoder createAnalogInputEncoder(String resourceId) {
    try {
      EncoderConfig config = configUtils.readFromClassPath(EncoderConfig.class, resourceId);
      AnalogInputEncoder encoder = new AnalogInputEncoder(new AnalogInput(config.getPort()), config);
      return encoder;
    } catch(IOException e) {
      throw new RuntimeException("Error Initialzing AnalongInputEncoder", e);
    }
  }

  public ShiftingSwerveModule initSwerveModule(String speedConfig, String angleConfig, String encoderConfig, String moduleConfig){
    // SOTAMotorController speedMotor = initFalconDelegate(speedConfig);
    // SOTAMotorController angleMotor = initSparkMaxDelegate(angleConfig);
    // SOTAEncoder encoder = new AnalogInputEncoder();
    // SOTAMotorController angleComposite = new CompositeMotor(angleMotor, encoder);

    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, moduleConfig);
      SOTAMotorController speedMotor = createFalcon(speedConfig);
      SOTAMotorController angleMotor = createCompositeSparkMax(angleConfig, encoderConfig);
      return new ShiftingSwerveModule(angleMotor, speedMotor, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
    
  }
}
