// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.ResetExtension;
import frc.robot.Commands.RotationPID;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.SuperStructure;
import frc.robot.Subsystems.Swerve.DoubleSolenoidShifter;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;
import frc.robot.util.ConfigUtils;
import lib.Config.DoubleSolenoidConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.ShiftingSwerveDriveConfig;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.Config.SuperStructureConfig;
import lib.Control.SOTAXboxcontroller;
import lib.Factories.MotorControllerFactory;
import lib.Gyro.NavX;
import lib.Gyro.SOTAGyro;
import lib.MotorController.SOTAMotorController;
import lib.Pneumatics.GearShifter;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;

  private final SOTAXboxcontroller dController;
  private final SOTAXboxcontroller mController;

  private ShiftingSwerveDrive mSwerveDrive;
  private Rotation mRotation;
  private Extension mExtension;
  private DefaultDrive mDriveCommand;
  private RotationPID rotationPID;
  private ExtensionPID extensionPID;
  private ResetExtension mResetExtension;

  public RobotContainer() {
    
    SmartDashboard.putNumber("rotation setpoint", 180);
    SmartDashboard.putNumber("Extension Length", 0);

    SmartDashboard.putNumber("set p", 0);
    SmartDashboard.putNumber("TestDelta", 0.0);

    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);

    dController = new SOTAXboxcontroller(0);
    mController = new SOTAXboxcontroller(1);

    ShiftingSwerveModule[] swerveModules = {
      initSwerveModule(
        "Swerve/FrontLeft/SpeedFalcon",
        "Swerve/FrontLeft/AngleSparkMax",
        "Swerve/FrontLeft/ShiftingSwerveModule"
      ),

      initSwerveModule(
        "Swerve/BackLeft/SpeedFalcon",
        "Swerve/BackLeft/AngleSparkMax",
      "Swerve/BackLeft/ShiftingSwerveModule"
      ),

      initSwerveModule(
        "Swerve/BackRight/SpeedFalcon",
        "Swerve/BackRight/AngleSparkMax",
        "Swerve/BackRight/ShiftingSwerveModule"
      ),
      
      initSwerveModule(
        "Swerve/FrontRight/SpeedFalcon",
        "Swerve/FrontRight/AngleSparkMax",
        "Swerve/FrontRight/ShiftingSwerveModule"
      ),
    };
    
    try{
      SOTAGyro gyro = new NavX(new AHRS(Port.kMXP));
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
      GearShifter shifter = new DoubleSolenoidShifter(solenoid, 
        configUtils.readFromClassPath(DoubleSolenoidConfig.class, 
        "Swerve/DoubleSolenoidSwerveShifter"));
      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro,
       configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "Swerve/ShiftingSwerveDrive"));

    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Faild to create swerveDrive", e);
    }
   


    try{
    SOTAMotorController rotationMotor = MotorControllerFactory.generateSparkDelegate(
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/RotatorMotor")));
    SOTAMotorController winchMotor = MotorControllerFactory.generateSparkDelegate
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/WinchMotor"));

    DigitalInput limitSwitch = new DigitalInput(0);
      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class, "SuperStructure/SuperStructure");
    this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotationMotor, superStructureConfig);
      SuperStructure superStructure = new SuperStructure(mExtension::getLength,mRotation::getRotationDegrees, superStructureConfig);
      PIDController armRotationController = new PIDController(0.03,0,0);
      ProfiledPIDController extensController = new ProfiledPIDController(3, 0, 0,
       new TrapezoidProfile.Constraints(40.0,80.0));

      this.rotationPID = new RotationPID(mRotation, armRotationController, 180, mController, mExtension::getLength, superStructure::minRotation, superStructure::maxRotation);
      this.extensionPID = new ExtensionPID(extensController, mExtension,  mController, superStructure::maxExtension);
      this.mResetExtension = new ResetExtension(mExtension);
    
    } catch(IOException e){}

    


    this.mDriveCommand = new DefaultDrive(mSwerveDrive, dController);

    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    

    mSwerveDrive.setDefaultCommand(mDriveCommand);
    mExtension.setDefaultCommand(extensionPID);
    mRotation.setDefaultCommand(rotationPID);
  }

  private void configureBindings() {
    // TODO: add reset gyro
    mController.a().onTrue(mResetExtension);
    dController.x().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(true);
    }));
    dController.y().onTrue(new InstantCommand(() -> {
      mSwerveDrive.setFieldCentricActive(false);
    }));
    
      
  }

  public Command getAutonomousCommand() {
    return null;
  }

  

  
  

  public ShiftingSwerveModule initSwerveModule(String speedConfig, String angleConfig, String moduleConfig){

    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, moduleConfig);
      MotorControllerConfig speedMotorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, speedConfig);
      MotorControllerConfig rotatorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, angleConfig);
      SOTAMotorController speedMotor = MotorControllerFactory.generateFalconDelegate(speedMotorConfig);
      SOTAMotorController angleMotor = MotorControllerFactory.generateSparkDelegate(rotatorConfig);
      return new ShiftingSwerveModule(angleMotor, speedMotor, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
  }
}
