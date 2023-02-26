// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Gyro.NavX;
import frc.robot.Subsystems.Swerve.DoubleSolenoidSwerveShifter;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;
import frc.robot.util.ConfigUtils;
import lib.Config.DoubleSolenoidSwerveShifterConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.ShiftingSwerveDriveConfig;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.MotorController.FalconDelegate;
import lib.MotorController.SparkMaxDelegate;

import static frc.robot.Constants.Pneumatics.*;
import static frc.robot.Constants.Swerve.*;

import frc.robot.Commands.DefaultDrive;

public class RobotContainer {

  private ShiftingSwerveDrive mSwerveDrive;

  private CommandXboxController mDriveStick;
  
  // private final ConfigUtils configUtils;

  public RobotContainer() throws Exception {
    // For testing do dont wanna make a hard coded controller
    mDriveStick = new CommandXboxController(0);

    // TODO: add this back in when json issue is resolved
    // ObjectMapper mapper = new ObjectMapper();
    // mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    // configUtils = new ConfigUtils(mapper);

    ShiftingSwerveModule[] shiftingSwerveModules = createSwerveModules();
    DoubleSolenoidSwerveShifter shifter =
      new DoubleSolenoidSwerveShifter(
        new DoubleSolenoid(MODULE_TYPE, SWERVE_FORWARD, SWERVE_REVERSE),
        // configUtils.readConfigFromClasspath("Swerve/DoubleSolenoidSwerveShifter.json", DoubleSolenoidSwerveShifterConfig.class)
        new DoubleSolenoidSwerveShifterConfig("FORWARD", "REVERSE")
    );
    NavX gyro = new NavX(new AHRS(Port.kMXP));            

    mSwerveDrive = new ShiftingSwerveDrive(
      shiftingSwerveModules,
      shifter,
      gyro,
      // configUtils.readConfigFromClasspath("Swerve/ShiftingSwerve.json", ShiftingSwerveDriveConfig.class)
      new ShiftingSwerveDriveConfig(
        19.625, 
        19.625, 
        5.2, 
        15
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    mSwerveDrive.setDefaultCommand(
      new RunCommand(() -> {
        double fwd = mDriveStick.getLeftY();
        double str = mDriveStick.getLeftX();
        double rot = mDriveStick.getRightX();
    
        // Squares inputs and preserves sign TODO: make controller class that handles this
        fwd = -Math.signum(fwd) * fwd * fwd;
        str = -Math.signum(str) * str * str;
        rot = -Math.signum(rot) * rot * rot;

        SmartDashboard.putNumber("fwd", fwd);
        SmartDashboard.putNumber("str", str);
        SmartDashboard.putNumber("rot", rot);


        mSwerveDrive.drive(fwd, str, rot, mSwerveDrive.getRotation2d(), new Translation2d());

      }, mSwerveDrive)

    );
  }

  private ShiftingSwerveModule[] createSwerveModules() throws Exception {

    //TODO: temp remove this once json problem is fixed
    ShiftingSwerveModuleConfig frontRightConfig = new ShiftingSwerveModuleConfig(
      5.07,
      10.04, 
      0.0, 
      5.0, 
      4.0,
      0.0001,
      0.0,
      0.0,
      0.05,
      1000.0,
      2000.0,
      0.6284,
      0.0005339,
      1.0,
      0.0,
      0.0,
      0.0,
      70.0,
      0.6284, 
      0.0005339
    );
    MotorControllerConfig frontRightSpeedConfig = new MotorControllerConfig(
      false, 
      "BRAKE", 
      "BRUSHLESS", 
      2048
    );
    MotorControllerConfig frontRightAngleConfig = new MotorControllerConfig(
      false,
      "BRAKE",
      "BRUSHLESS",
      48
    );
    ShiftingSwerveModuleConfig frontLeftConfig = new ShiftingSwerveModuleConfig(
      5.07,
      10.04, 
      0.0, 
      5.0, 
      4.0,
      0.0001,
      0.0,
      0.0,
      0.05,
      1000.0,
      2000.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      70.0,
      0.0, 
      0.0
    );
    MotorControllerConfig frontLeftSpeedConfig = new MotorControllerConfig(
      false, 
      "BRAKE", 
      "BRUSHLESS", 
      2048
    );
    MotorControllerConfig frontLeftAngleConfig = new MotorControllerConfig(
      false,
      "BRAKE",
      "BRUSHLESS",
      48
    );
    ShiftingSwerveModuleConfig backRightConfig = new ShiftingSwerveModuleConfig(
      5.07,
      10.04, 
      0.0, 
      5.0, 
      4.0,
      0.0001,
      0.0,
      0.0,
      0.05,
      1000.0,
      2000.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      70.0,
      0.0, 
      0.0
    );
    MotorControllerConfig backRightSpeedConfig = new MotorControllerConfig(
      false, 
      "BRAKE", 
      "BRUSHLESS", 
      2048
    );
    MotorControllerConfig backRightAngleConfig = new MotorControllerConfig(
      false,
      "BRAKE",
      "BRUSHLESS",
      48
    );
    ShiftingSwerveModuleConfig backLeftConfig = new ShiftingSwerveModuleConfig(
      5.07,
      10.04, 
      0.0, 
      5.0, 
      4.0,
      0.0001,
      0.0,
      0.0,
      0.05,
      1000.0,
      2000.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      70.0,
      0.0, 
      0.0
    );
    MotorControllerConfig backLeftSpeedConfig = new MotorControllerConfig(
      false, 
      "BRAKE", 
      "BRUSHLESS", 
      2048
    );
    MotorControllerConfig backLeftAngleConfig = new MotorControllerConfig(
      false,
      "BRAKE",
      "BRUSHLESS",
      48
    );
  

    ShiftingSwerveModule[] shiftingSwerveModules = {
      // Front Right
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_FRONT_RIGHT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/FrontRight/SpeedFalcon.json", MotorControllerConfig.class)
          frontRightSpeedConfig
        ),
        new SparkMaxDelegate(
          ANGLE_FRONT_RIGHT_PORT,
          // configUtils.readConfigFromClasspath("Serve/FrontRight/AngleSparkMax.json", MotorControllerConfig.class)
          frontRightAngleConfig
        ),
        new AnalogInput(ANGLE_ENCODER_FRONT_RIGHT_PORT),
        // configUtils.readConfigFromClasspath("Swerve/FrontRight/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
        frontRightConfig
      ),
      // Front Left
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_FRONT_LEFT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/FrontLeft/SpeedFalcon.json", MotorControllerConfig.class)
          frontLeftSpeedConfig
        ),
        new SparkMaxDelegate(
          ANGLE_FRONT_LEFT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/FrontLeft/AngleSparkMax.json", MotorControllerConfig.class)
          frontLeftAngleConfig
        ),
        new AnalogInput(ANGLE_ENCODER_FRONT_LEFT_PORT),
        // configUtils.readConfigFromClasspath("Swerve/FrontLeft/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
        frontLeftConfig
      ),
      // Back Right
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_BACK_RIGHT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/BackRight/SpeedFalcon.json", MotorControllerConfig.class)
          backRightSpeedConfig
        ),
        new SparkMaxDelegate(
          ANGLE_BACK_RIGHT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/BackRight/AngleSparkMax.json", MotorControllerConfig.class)
          backRightAngleConfig
        ),
        new AnalogInput(ANGLE_ENCODER_BACK_RIGHT_PORT),
        // configUtils.readConfigFromClasspath("Swerve/BackRight/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
        backRightConfig
      ),
      // Back Left
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_BACK_LEFT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/BackLeft/SpeedFalcon.json", MotorControllerConfig.class)
          backLeftSpeedConfig
        ),
        new SparkMaxDelegate(
          ANGLE_BACK_LEFT_PORT,
          // configUtils.readConfigFromClasspath("Swerve/BackLeft/AngleSparkMax.json", MotorControllerConfig.class)
          backLeftAngleConfig
        ),
        new AnalogInput(ANGLE_ENCODER_BACK_LEFT_PORT),
        // configUtils.readConfigFromClasspath("Swerve/BackLeft/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
        backLeftConfig
      )
    };
    return shiftingSwerveModules;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
