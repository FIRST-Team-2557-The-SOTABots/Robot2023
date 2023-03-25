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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.BasicIntakeCommand;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.Autos.BackUpMobility;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
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
import lib.Control.SOTA_Xboxcontroller;
import lib.Factories.MotorControllerFactory;
import lib.Gyro.NavX;
import lib.Gyro.SOTA_Gyro;
import lib.MotorController.SOTA_MotorController;
import lib.Pneumatics.GearShifter;

public class RobotContainer {
  // private final ArmInterface arm;
  private final ConfigUtils configUtils;

  private final SOTA_Xboxcontroller dController;
  private final SOTA_Xboxcontroller mController;

  private ShiftingSwerveDrive mSwerveDrive;
  private Rotation mRotation;
  private Extension mExtension;
  private Intake mIntake;

  private DefaultDrive mDriveCommand;
  private RotationPID rotationPID;
  private ExtensionPID extensionPID;
  private BasicIntakeCommand intakeCommand;
  // private ResetExtension mResetExtension;
  // private AutoLevel mAutoLevel;
  private BackUpMobility mBackUpMobility;
  private ParallelDeadlineGroup mBackUpAuto;
  // private SendableChooser<CommandBase> mAutoChooser;

  public RobotContainer() {
    
    // SmartDashboard.putNumber("rotation setpoint", 180);
    // SmartDashboard.putNumber("Extension Length", 0);

    // SmartDashboard.putNumber("set p", 0);
    // SmartDashboard.putNumber("TestDelta", 0.0);

    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);
    
    // mAutoChooser = new SendableChooser<CommandBase>();
    // mAutoChooser.setDefaultOption("None", null);
    // mAutoChooser.addOption("Back Up Mobility", 
    //   new RunCommand(() -> {
    //     mSwerveDrive.drive(-0.5, 0, 0, mSwerveDrive.getRotation2d());
    //   }, mSwerveDrive).withTimeout(2.5)
    // );

    dController = new SOTA_Xboxcontroller(0);
    mController = new SOTA_Xboxcontroller(1);

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
      SOTA_Gyro gyro = new NavX(new AHRS(Port.kMXP));//TODO: change back to SOTAGyro
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
      GearShifter shifter = new DoubleSolenoidShifter(solenoid, 
        configUtils.readFromClassPath(DoubleSolenoidConfig.class, 
        "Swerve/DoubleSolenoidSwerveShifter"));
      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro, 
       configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "Swerve/ShiftingSwerveDrive"));

      //  mAutoLevel = new AutoLevel(mSwerveDrive);

    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Faild to create swerveDrive", e);
    }
   

    try{
    SOTA_MotorController rotationMotor = MotorControllerFactory.generateSparkDelegate(
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/RotatorMotor")));

    SOTA_MotorController winchMotor = MotorControllerFactory.generateSparkDelegate
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/WinchMotor"));

    // MotorController intakeMotorRight = new CANSparkMax(2, MotorType.kBrushless);
    // intakeMotorRight.setInverted(true);
    // MotorControllerGroup intakeMotors = new MotorControllerGroup(new CANSparkMax(1, MotorType.kBrushless),
    // intakeMotorRight);
    SOTA_MotorController intakeMotorBottom = MotorControllerFactory.generateSparkDelegate
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/IntakeMotorBottom"));
    SOTA_MotorController intakeMotorTop = MotorControllerFactory.generateSparkDelegate
    (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/IntakeMotorTop"));

    // Using SOTA_MotorController as a MotorController limits it's capabilities but it has immense compatibility
    MotorControllerGroup intakeMotors = new MotorControllerGroup(intakeMotorBottom, intakeMotorTop);

    DigitalInput limitSwitch = new DigitalInput(0);

      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
       "SuperStructure/SuperStructure");

      this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotationMotor, superStructureConfig);
      this.mIntake = new Intake(intakeMotors);

      SuperStructure superStructure = new SuperStructure(mExtension::getLength,mRotation::getRotationDegrees, superStructureConfig);
      PIDController armRotationController = new PIDController(0.03,0,0);

      ProfiledPIDController extensController = new ProfiledPIDController(3, 0, 0,
       new TrapezoidProfile.Constraints(40.0,80.0));

      this.rotationPID = new RotationPID(mRotation, armRotationController, 150, mExtension::getLengthFromStart, superStructure::minRotation, superStructure::maxRotation, superStructureConfig);
      this.extensionPID = new ExtensionPID(extensController, mExtension, superStructure::maxExtension);
      // this.mResetExtension = new ResetExtension(mExtension);
      this.intakeCommand = new BasicIntakeCommand(mIntake, mController);
    
    } catch(IOException e){}

    this.mDriveCommand = new DefaultDrive(mSwerveDrive, dController);

    mBackUpMobility = new BackUpMobility(mSwerveDrive);

    mBackUpAuto = CommandGroupBase.deadline(
      new WaitCommand(mBackUpMobility.kTime),
      mBackUpMobility
    );    

    configureDefaultCommands();
    configureBindings();
    
  }

  private void configureDefaultCommands(){
    mSwerveDrive.setDefaultCommand(mDriveCommand);
    mExtension.setDefaultCommand(extensionPID);
    mRotation.setDefaultCommand(rotationPID);
    mIntake.setDefaultCommand(intakeCommand);

  }

  private void configureBindings() {

    dController.start().onTrue(
      new InstantCommand(
        () -> {
          mSwerveDrive.resetGyro();
        },
        mSwerveDrive
      )
    );
    dController.a().onTrue(
      new InstantCommand(
        () -> {
          mSwerveDrive.setFieldCentricActive(true);
        },
        mSwerveDrive
      )
    );
    dController.b().onTrue(
      new InstantCommand(
        () -> {
          mSwerveDrive.setFieldCentricActive(false);
        },
        mSwerveDrive
      )
    );

    // mController.rightTrigger().whileTrue(new RunCommand(mIntake::intake, mIntake)).onFalse(new SequentialCommandGroup(
    //   new RunCommand(() -> mIntake.set(-0.01), mIntake).withTimeout(0.5),
    //   new InstantCommand(mIntake::stop)));
    // mController.leftTrigger().whileTrue(new RunCommand(mIntake::release, mIntake)).onFalse(new InstantCommand(mIntake::stop));
    mController.leftBumper().onTrue( // Substation
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.SUBSTATION);
          extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
        },
        mRotation, mExtension
      )
    );
    mController.back().onTrue( // FLOOR
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.FLOOR);
          extensionPID.setSetpoint(ExtensionSetpoint.MID);
        },
        mRotation, mExtension
      )
    );
    mController.a().onTrue( // SCORE MID ON RELEASE HIGH
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.SCORE);
          extensionPID.setSetpoint(ExtensionSetpoint.MID);
        },
        mRotation, mExtension
      )
    ).onFalse(
      new InstantCommand(
        () -> {
          extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
        },
        mExtension
      )
    );
    mController.x().onTrue( // RESET
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.RESET);
          extensionPID.setSetpoint(ExtensionSetpoint.RESET);
        },
        mRotation, mExtension
      )
    );
    

  }

  public Command getAutonomousCommand() {
    return mBackUpAuto;
    // return mBackUpMobility.withTimeout(mBackUpMobility.kTime); // removed use of depracated thingy if dont work just comment out
  }

  public ShiftingSwerveModule initSwerveModule(String speedConfig, String angleConfig, String moduleConfig) {
    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, moduleConfig);
      MotorControllerConfig speedMotorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, speedConfig);
      MotorControllerConfig rotatorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, angleConfig);
      SOTA_MotorController speedMotor = MotorControllerFactory.generateFalconDelegate(speedMotorConfig);
      SOTA_MotorController angleMotor = MotorControllerFactory.generateSparkDelegate(rotatorConfig);
      return new ShiftingSwerveModule(angleMotor, speedMotor, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
  }
}
