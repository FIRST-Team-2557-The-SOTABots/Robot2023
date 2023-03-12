// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import javax.swing.plaf.nimbus.State;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.ArmPID;
import frc.robot.Commands.ArmPID2;
import frc.robot.Commands.BasicArmExtension;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModuleI2;
import frc.robot.Util.Configs.DoubleSolenoidConfig;
import frc.robot.Util.Configs.MotorControllerConfig;
import frc.robot.Util.Configs.ShiftingSwerveDriveConfig;
import frc.robot.Util.Configs.ShiftingSwerveModuleConfig;
import frc.robot.Util.Configs.SuperStructureConfig;
import frc.robot.Util.Controllers.AnalogInputEncoder;
import frc.robot.Util.Controllers.CompositeMotor;
import frc.robot.Util.Controllers.DoubleSolenoidShifter;
import frc.robot.Util.Controllers.FalconDelegate;
import frc.robot.Util.Controllers.MotorLimits;
import frc.robot.Util.Controllers.NavX;
import frc.robot.Util.Controllers.PigeonDelegate;
import frc.robot.Util.Controllers.SOTADutyCycleEncoder;
import frc.robot.Util.Controllers.SOTAMotorControllerGroup;
import frc.robot.Util.Controllers.SOTAXboxcontroller;
import frc.robot.Util.Controllers.SparkMaxDelegate;
import frc.robot.Util.Interfaces.GearShifter;
import frc.robot.Util.Interfaces.SOTAEncoder;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;
import frc.robot.Util.UtilityClasses.ConfigUtils;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;

  private final SOTAGyro gyro;

  private final SOTAXboxcontroller dController;
  private final SOTAXboxcontroller mController;

  private ShiftingSwerveDrive mSwerveDrive;
  // private SuperStructure mArm;
  private Extension mExtension;
  private Rotation mRotation;
  private Intake mIntake;

  private ArmPID2 rotationPID;
  private DriveCommand mDriveCommand;
  private BasicArmExtension armExtensionCommand;
  private ExtensionPID extensionPID;

  public RobotContainer() {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new SOTAXboxcontroller(0);
    mController = new SOTAXboxcontroller(1);

    this.gyro = new NavX(new AHRS(Port.kMXP));

    ShiftingSwerveModuleI2[] swerveModules = {
      initSwerveModule("Swerve/FrontLeft/SpeedFalcon",
        "Swerve/FrontLeft/AngleSparkMax",
       "Swerve/FrontLeft/ShiftingSwerveModule"),

      initSwerveModule("Swerve/BackLeft/SpeedFalcon",
        "Swerve/BackLeft/AngleSparkMax",
       "Swerve/BackLeft/ShiftingSwerveModule"),

       initSwerveModule("Swerve/BackRight/SpeedFalcon",
       "Swerve/BackRight/AngleSparkMax",
      "Swerve/BackRight/ShiftingSwerveModule"),
      
       initSwerveModule("Swerve/FrontRight/SpeedFalcon",
       "Swerve/FrontRight/AngleSparkMax",
      "Swerve/FrontRight/ShiftingSwerveModule"),
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
    MotorLimits rotatorMotorLimits = new MotorLimits(0.345, 0.8); //TODO: 
    MotorLimits extendLimit = new MotorLimits(null, 630.0);
    SOTAMotorController winchMotor = initSparkMaxDelegate("SuperStructure/WinchMotor", extendLimit);
    SOTAMotorController rotatorMotor = initSparkMaxDelegate("SuperStructure/RotatorMotor");
    SOTAEncoder rotatorEncoder = new SOTADutyCycleEncoder(1);
    SOTAMotorController rotatorComposite = new CompositeMotor(rotatorMotor, rotatorEncoder, rotatorMotorLimits);
    SOTAGyro armGyro = new PigeonDelegate(4);
    DigitalInput limitSwitch = new DigitalInput(3);
    // SOTAMotorController leftMotorIntake = initSparkMaxDelegate("SuperStructure/IntakeMotorLeft");
    // SOTAMotorController rightMotorIntake = initSparkMaxDelegate("SuperStructure/IntakeMotorRight");
    // SOTAMotorController intakeMotors = new SOTAMotorControllerGroup(rightMotorIntake, leftMotorIntake);
    // intakeMotors.setInverted(true);

    MotorController leftMotor = new CANSparkMax(7, MotorType.kBrushless);
    MotorController rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    leftMotor.setInverted(true);
    MotorController intakeMotors = new MotorControllerGroup(rightMotor, leftMotor);

    this.mIntake = new Intake(intakeMotors);
    TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(40, 80);

    ProfiledPIDController extensController = new ProfiledPIDController(0.8, 0, 0, trapezoidProfile);
    PIDController armRotationController = new PIDController(0.03,0,0);

    try{
      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class, "SuperStructure/SuperStructure");
      this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotatorComposite, armGyro, superStructureConfig);
      SuperStructure superStructure = new SuperStructure(mExtension::getEncoder,mRotation::getRotationDegrees, superStructureConfig);
      this.rotationPID = new ArmPID2( mRotation, armRotationController, 0, mController
      , superStructure::minRotation, superStructure::maxRotation);
      this.extensionPID = new ExtensionPID(extensController, mExtension, mController, superStructure::maxExtension);
    } catch(IOException e){
      e.printStackTrace();
      throw new RuntimeException("Faild to create SuperStructure", e);      
    }

    // this.mArm = new SuperStructure(armGyro, winchMotor, rotatorComposite, limitSwitch, intakeMotors);

    this.mDriveCommand = new DriveCommand(mSwerveDrive, dController);


    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    SmartDashboard.putNumber("rotation setpoint", 0.0);
    SmartDashboard.putNumber("Extension Length", 0.0);


    mSwerveDrive.setDefaultCommand(mDriveCommand);

    mExtension.setDefaultCommand(extensionPID);

    mRotation.setDefaultCommand(rotationPID);
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

    mController.x().whileTrue(new RunCommand(mIntake::intake, mIntake)).onFalse(new InstantCommand(mIntake::stop));
    mController.y().whileTrue(new RunCommand(mIntake::release, mIntake)).onFalse(new InstantCommand(mIntake::stop));
    
  }

  public Command getAutonomousCommand() {
    return new RunCommand(() ->{
      // mSwerveDrive.drive(1, 0, 0, gyro.getRotation2d(), new Translation2d());
    }, mSwerveDrive);
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
    return initSparkMaxDelegate(resourceId, new MotorLimits());
  }


  public SparkMaxDelegate initSparkMaxDelegate(String resourceId, MotorLimits limits){
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
      return new SparkMaxDelegate(motor, limits, config.getCountsPerRevolution());
    } catch(IOException e){
      throw new RuntimeException("Error Initialzing SparkMax", e);
    }
  }

  public ShiftingSwerveModuleI2 initSwerveModule(String speedConfig, String angleConfig,  String ModuleConfig){
    // SOTAMotorController speedMotor = initFalconDelegate(speedConfig);
    // SOTAMotorController angleMotor = initSparkMaxDelegate(angleConfig);
    // SOTAEncoder encoder = new AnalogInputEncoder();
    // SOTAMotorController angleComposite = new CompositeMotor(angleMotor, encoder);

    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, ModuleConfig);
      SOTAMotorController speedMotor = initFalconDelegate(speedConfig);
    SOTAMotorController angleMotor = initSparkMaxDelegate(angleConfig);
    SOTAEncoder encoder = new AnalogInputEncoder(config.getEncoderPort());
    SOTAMotorController angleComposite = new CompositeMotor(angleMotor, encoder);
      return new ShiftingSwerveModuleI2(angleComposite,speedMotor, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
    
  }
}
