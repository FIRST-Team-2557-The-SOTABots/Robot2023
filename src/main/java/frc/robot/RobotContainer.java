// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.function.IntSupplier;

import javax.management.openmbean.OpenDataException;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AutoLevel;
import frc.robot.Commands.BasicIntakeCommand;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.ResetExtension;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Commands.Autos.BackUpMobility;
import frc.robot.Commands.Autos.OnePieceMobilityAutoBalence;
import frc.robot.Commands.Autos.PlaceCondAndMobilityWithPath;
import frc.robot.Commands.Autos.PlaceCone;
import frc.robot.Commands.Autos.PlaceConeAndMobility;
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
import lib.Factories.AutoFactory;
import lib.Factories.MotorControllerFactory;
import lib.Gyro.NavX;
import lib.Gyro.SOTA_Gyro;
import lib.MotorController.SOTA_MotorController;
import lib.MotorController.SparkMaxDelegate;
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

  private SuperStructure superStructure;

  private DefaultDrive mDriveCommand;
  private RotationPID rotationPID;
  private ExtensionPID extensionPID;
  private BasicIntakeCommand intakeCommand;
  private ResetExtension mResetExtension;
  private AutoLevel mAutoLevel;
  private BackUpMobility mBackUpMobility;
  private ParallelDeadlineGroup mBackUpAuto;

  

  private SwerveAutoBuilder mAutoBuilder;

  private SendableChooser<Command> mAutoChooser;

  

  // private SendableChooser<CommandBase> mAutoChooser;

  public RobotContainer() {
 

    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    this.configUtils = new ConfigUtils(mapper);
    
 

    dController = new SOTA_Xboxcontroller(0);
    mController = new SOTA_Xboxcontroller(1);

    
    try{
      SOTA_Gyro gyro = new NavX(new AHRS(Port.kMXP), true);
      DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
      GearShifter shifter = new DoubleSolenoidShifter(solenoid, 
        configUtils.readFromClassPath(DoubleSolenoidConfig.class, 
        "Swerve/DoubleSolenoidSwerveShifter"));

        ShiftingSwerveModule[] swerveModules = {
      
          initSwerveModule(
            "Swerve/FrontLeft/SpeedFalcon",
            "Swerve/FrontLeft/AngleSparkMax",
            "Swerve/FrontLeft/ShiftingSwerveModule",
            shifter::getGear
          ),
    
          initSwerveModule(
            "Swerve/BackLeft/SpeedFalcon",
            "Swerve/BackLeft/AngleSparkMax",
          "Swerve/BackLeft/ShiftingSwerveModule",
          shifter::getGear
          ),
    
          initSwerveModule(
            "Swerve/BackRight/SpeedFalcon",
            "Swerve/BackRight/AngleSparkMax",
            "Swerve/BackRight/ShiftingSwerveModule",
            shifter::getGear
          ),
          
          initSwerveModule(
            "Swerve/FrontRight/SpeedFalcon",
            "Swerve/FrontRight/AngleSparkMax",
            "Swerve/FrontRight/ShiftingSwerveModule",
            shifter::getGear
          ),
        };

      mSwerveDrive = new ShiftingSwerveDrive(swerveModules, shifter, gyro, 
       configUtils.readFromClassPath(ShiftingSwerveDriveConfig.class, "Swerve/ShiftingSwerveDrive"));

       
       mAutoLevel = new AutoLevel(mSwerveDrive);


    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("Faild to create swerveDrive", e);
    }
   

    try {
      SOTA_MotorController rotationMotor = MotorControllerFactory.generateSparkDelegate(
      (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/RotatorMotor")));

      SOTA_MotorController winchMotor = MotorControllerFactory.generateSparkDelegate
      (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/WinchMotor"));

      SOTA_MotorController intakeMotorTop = MotorControllerFactory.generateSparkDelegate
      (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/IntakeMotorTop"));
      SOTA_MotorController intakeMotorBottom = MotorControllerFactory.generateSparkDelegate
      (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/IntakeMotorBottom"));
      // CANSparkMax intakeMotor1 = new CANSparkMax(1, MotorType.kBrushless);
      // CANSparkMax intakeMotor2 = new CANSparkMax(2, MotorType.kBrushless);
      // intakeMotor1.setInverted(true);

      // SOTA_MotorController intakeMotorBottom = new SparkMaxDelegate(intakeMotor1, null);
      // SOTA_MotorController intakeMotorTop = new SparkMaxDelegate(intakeMotor2, null);

      DigitalInput limitSwitch = new DigitalInput(0);

      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
       "SuperStructure/SuperStructure");

      this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotationMotor, superStructureConfig);
      this.mIntake = new Intake(intakeMotorTop, intakeMotorBottom);

      this.superStructure = new SuperStructure(mExtension::getLength, mRotation::getRotationDegrees, superStructureConfig);


      ProfiledPIDController extensController = new ProfiledPIDController(3, 0, 0,
       new TrapezoidProfile.Constraints(40.0,80.0));

      this.rotationPID = new RotationPID(mRotation, mExtension::getLengthFromStart, superStructure::minRotation, superStructure::maxRotation, superStructureConfig);
      this.extensionPID = new ExtensionPID(extensController, mExtension, superStructure::maxExtension);
      this.mResetExtension = new ResetExtension(mExtension);
      this.intakeCommand = new BasicIntakeCommand(mIntake, mController::getLeftY);
    
    } catch(IOException e){}

    this.mDriveCommand = new DefaultDrive(mSwerveDrive, dController);

    mBackUpMobility = new BackUpMobility(mSwerveDrive);

    mBackUpAuto = CommandGroupBase.deadline(
      new WaitCommand(mBackUpMobility.kTime),
      mBackUpMobility
    );    

    Map<String, Command> eventMap = new HashMap<String, Command>();
      eventMap.put("event1", new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.FLOOR);
          extensionPID.setSetpoint(ExtensionSetpoint.MID);
        },
        mRotation, mExtension
      ));
      eventMap.put("event2", new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.MID);
          extensionPID.setSetpoint(ExtensionSetpoint.MID);
        },
        mRotation, mExtension
      ));
      mAutoBuilder = AutoFactory.swerveAutoBuilderGenerator(mSwerveDrive, eventMap);

    configureAutos();

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
        mSwerveDrive::resetGyro
      ) 
    );
    // dController.back().onTrue(new InstantCommand(() -> {
    //   mSwerveDrive.updatePose(new Pose2d());
    // }));
    dController.rightBumper().onTrue(
      new InstantCommand(
        () -> {
          mSwerveDrive.setFieldCentricActive(true);
        },
        mSwerveDrive
      )
    );
    dController.leftBumper().onTrue(
      new InstantCommand(
        () -> {
          mSwerveDrive.setFieldCentricActive(false);
        },
        mSwerveDrive
      )
    );
    // dController.y().whileTrue(mAutoLevel);

    // dController.leftTrigger().onTrue(
    //   new InstantCommand(
    //     () -> {
    //       mSwerveDrive.shift(0);
    //     }
    //   )
    // );
    // dController.rightTrigger().onTrue(
    //   new InstantCommand(
    //     () -> {
    //       mSwerveDrive.shift(1);
    //     }
    //   )
    // );

    mController.b().whileTrue(new InstantCommand(() -> {
      mIntake.set(-5);
    }));
    mController.leftBumper().onTrue( // Substation
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.SUBSTATION);
          extensionPID.setSetpoint(ExtensionSetpoint.SUBSTATION);
        },
        mRotation, mExtension
      )
    );
    mController.rightBumper().onTrue(
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.REST);
          extensionPID.setSetpoint(ExtensionSetpoint.REST);
        }
      )
    );
    mController.back().onTrue( // FLOOR
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.FLOOR);
          extensionPID.setSetpoint(ExtensionSetpoint.FLOOR);
        },
        mRotation, mExtension
      )
    );
    mController.start().onTrue(
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.FLOORCONEKNOCK);
          extensionPID.setSetpoint(ExtensionSetpoint.FLOORCONE);
        }
      )
    ).onFalse(
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.FLOORCONE);
        },
        mRotation
      )
    );
    mController.a().onTrue( // SCORE MID ON RELEASE HIGH
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.MID);
          extensionPID.startExhaustTimeout();
          extensionPID.setSetpoint(ExtensionSetpoint.MID);
        },
        mRotation, mExtension
      )
    ).onFalse(
      new InstantCommand(
        () -> {
          if (!extensionPID.exhaustTimedOut()) {
            rotationPID.setSetpoint(RotationSetpoint.HIGH);
            extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
          }
        },
        mExtension
      )
    );
    mController.b().onTrue(
      new InstantCommand(
        () -> {
          rotationPID.setSetpoint(RotationSetpoint.SINGLE);
          extensionPID.setSetpoint(ExtensionSetpoint.SINGLE);
        },
        mRotation, mExtension
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
    mController.y().onTrue(mResetExtension); // Emergency reset

  }

  public Command getAutonomousCommand() {
    return (Command) mAutoChooser.getSelected();
  }

  public ShiftingSwerveModule initSwerveModule(String speedConfig, String angleConfig, String moduleConfig, IntSupplier gear) {
    try{
      ShiftingSwerveModuleConfig config = configUtils.readFromClassPath(ShiftingSwerveModuleConfig.class, moduleConfig);
      MotorControllerConfig speedMotorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, speedConfig);
      MotorControllerConfig rotatorConfig = configUtils.readFromClassPath(MotorControllerConfig.class, angleConfig);
      SOTA_MotorController speedMotor = MotorControllerFactory.generateFalconDelegate(speedMotorConfig);
      SOTA_MotorController angleMotor = MotorControllerFactory.generateSparkDelegate(rotatorConfig);
      return new ShiftingSwerveModule(angleMotor, speedMotor, gear, config);
    } catch(IOException e){
      throw new RuntimeException("Could not create config", e);
    }
  }



  public void configureAutos(){
    try{
      SuperStructureConfig autoSuperStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
        "SuperStructure/SuperStructure");
      ProfiledPIDController autoExtensController = new ProfiledPIDController(3, 0, 0,
        new TrapezoidProfile.Constraints(40.0,80.0));
      RotationPID autorotationPID = new RotationPID(mRotation, mExtension::getLengthFromStart, superStructure::minRotation, superStructure::maxRotation, autoSuperStructureConfig);
      ExtensionPID autoextensionPID = new ExtensionPID(autoExtensController, mExtension, superStructure::maxExtension);
      ResetExtension automResetExtension = new ResetExtension(mExtension);
      BasicIntakeCommand autoIntakeCommand = new BasicIntakeCommand(mIntake, () -> 0.0);
      this.mAutoChooser = new SendableChooser<>();
      this.mAutoChooser.setDefaultOption("None", extensionPID);
      // this.mAutoChooser.addOption("Place And Mobility Path",
      //   new PlaceCondAndMobilityWithPath(mSwerveDrive, autoextensionPID, autorotationPID, mAutoBuilder, mIntake,
      //   PathPlanner.loadPath("Leave Community", new PathConstraints(4, 3.5))));
      this.mAutoChooser.addOption("Place", 
        new PlaceCone(autoextensionPID, autorotationPID, autoIntakeCommand));
      AutoLevel automAutoLevel = new AutoLevel(mSwerveDrive);

    // this.mAutoChooser.addOption("Place And Balance", new OnePieceMobilityAutoBalence(autoextensionPID, autorotationPID, mIntake, mSwerveDrive, automAutoLevel).withTimeout(15));
    
    SmartDashboard.putData("Auto", this.mAutoChooser);
  }catch(IOException e){}

  } 
}
