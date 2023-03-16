// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Subsystems.Swerve.DoubleSolenoidSwerveShifter;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;
import frc.robot.util.ConfigUtils;
import lib.Config.DoubleSolenoidSwerveShifterConfig;
import lib.Config.EncoderConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.ShiftingSwerveDriveConfig;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.Control.SOTAXboxcontroller;
import lib.Encoder.AnalogInputEncoder;
import lib.Encoder.SOTAEncoder;
import lib.Gyro.NavX;
import lib.Gyro.SOTAGyro;
import lib.MotorController.Falcon;
import lib.MotorController.SOTAMotorController;
import lib.MotorController.SparkMax;
import lib.Pneumatics.GearShifter;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;

  private final SOTAXboxcontroller dController;
  private final SOTAXboxcontroller mController;

  private ShiftingSwerveDrive mSwerveDrive;
  // private SuperStructure mArm;

  private DefaultDrive mDriveCommand;
  // private ArmPID2 mArmPID;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new SOTAXboxcontroller(0);
    mController = new SOTAXboxcontroller(1);

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
        "Swerve/FrontRight/AbsoluteEncoder",
        "Swerve/FrontRight/ShiftingSwerveModule"
      )
    };
    
    try{
      SOTAGyro gyro = new NavX(new AHRS(Port.kMXP));
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
      GearShifter shifter = new DoubleSolenoidSwerveShifter(solenoid, 
        configUtils.readFromClassPath(DoubleSolenoidSwerveShifterConfig.class, 
        "Swerve/DoubleSolenoidSwerveShifter"));
      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro, configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "Swerve/ShiftingSwerveDrive"));

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

    this.mDriveCommand = new DefaultDrive(mSwerveDrive, dController);

    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    mSwerveDrive.setDefaultCommand(mDriveCommand);
    // mArm.setDefaultCommand(mArmPID);
  }

  private void configureBindings() {

    dController.a().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(true);
    }));
    dController.b().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(false);
    }));
    dController.start().onTrue(new InstantCommand(() -> {
      mSwerveDrive.resetGyro();
    }));
    
      
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private Falcon createFalcon(String resourceId){
    try{
        MotorControllerConfig config = configUtils.readFromClassPath(MotorControllerConfig.class, resourceId);
        TalonFX motor = new TalonFX(config.getPort());
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
