// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystem.Gyro.NavX;
import frc.robot.Subsystem.Swerve.DoubleSolenoidSwerveShifter;
import frc.robot.Subsystem.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystem.Swerve.ShiftingSwerveModule;
import frc.robot.util.ConfigUtils;
import frc.robot.util.SwerveUtils;
import lib.Config.DoubleSolenoidSwerveShifterConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.ShiftingSwerveDriveConfig;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.MotorController.FalconDelegate;
import lib.MotorController.SparkMaxDelegate;

import static frc.robot.Constants.Pneumatics.*;
import static frc.robot.Constants.Swerve.*;

public class RobotContainer {

  private final ConfigUtils configUtils;

  public RobotContainer() throws Exception {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    configUtils = new ConfigUtils(mapper) ;
    SwerveUtils swerveUtils = new SwerveUtils();
    ShiftingSwerveModule[] shiftingSwerveModules = createSwerveModules();

    DoubleSolenoidSwerveShifter shifter =
      new DoubleSolenoidSwerveShifter(
        new DoubleSolenoid(MODULE_TYPE, SWERVE_FORWARD, SWERVE_REVERSE),
        configUtils.readConfigFromClasspath("Swerve/DoubleSolenoidSwerveShifter.json",
          DoubleSolenoidSwerveShifterConfig.class)
    );

    NavX gyro = new NavX(new AHRS(Port.kMXP));
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
    SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(kinematics, gyro.getAngleRotation2d(),
              swerveUtils.getModulePositions(shiftingSwerveModules));
    ShiftingSwerveDriveConfig swerveDriveConfig =
            configUtils.readConfigFromClasspath("Swerve/ShiftingSwerve.json", ShiftingSwerveDriveConfig.class);

    new ShiftingSwerveDrive(
      swerveUtils,
      shiftingSwerveModules,
      kinematics,
      swerveDriveOdometry,
      shifter,
      gyro,
      swerveDriveConfig.getMaxWheelSpeed()
    );
    configureBindings();
  }

  private ShiftingSwerveModule[] createSwerveModules() throws Exception {
      ShiftingSwerveModule[] shiftingSwerveModules = {
              // Front Right
              new ShiftingSwerveModule(
                      new FalconDelegate(
                              SPEED_FRONT_RIGHT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/FrontRight/SpeedFalcon.json", MotorControllerConfig.class)
                      ),
                      new SparkMaxDelegate(
                              ANGLE_FRONT_RIGHT_PORT,
                              configUtils.readConfigFromClasspath("Serve/FrontRight/AngleSparkMax.json", MotorControllerConfig.class)
                      ),
                      new AnalogInput(ANGLE_ENCODER_FRONT_RIGHT_PORT),
                      configUtils.readConfigFromClasspath("Swerve/FrontRight/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
              ),
              // Front Left
              new ShiftingSwerveModule(
                      new FalconDelegate(
                              SPEED_FRONT_LEFT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/FrontLeft/SpeedFalcon.json", MotorControllerConfig.class)
                      ),
                      new SparkMaxDelegate(
                              ANGLE_FRONT_LEFT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/FrontLeft/AngleSparkMax.json", MotorControllerConfig.class)
                      ),
                      new AnalogInput(ANGLE_ENCODER_FRONT_LEFT_PORT),
                      configUtils.readConfigFromClasspath("Swerve/FrontLeft/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
              ),
              // Back Right
              new ShiftingSwerveModule(
                      new FalconDelegate(
                              SPEED_BACK_RIGHT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/BackRight/SpeedFalcon.json", MotorControllerConfig.class)),
                      new SparkMaxDelegate(
                              ANGLE_BACK_RIGHT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/BackRight/AngleSparkMax.json", MotorControllerConfig.class)
                      ),
                      new AnalogInput(ANGLE_ENCODER_BACK_RIGHT_PORT),
                      configUtils.readConfigFromClasspath("Swerve/BackRight/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
              ),
              // Back Left
              new ShiftingSwerveModule(
                      new FalconDelegate(
                              SPEED_BACK_LEFT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/BackLeft/SpeedFalcon.json", MotorControllerConfig.class)
                      ),
                      new SparkMaxDelegate(
                              ANGLE_BACK_LEFT_PORT,
                              configUtils.readConfigFromClasspath("Swerve/BackLeft/AngleSparkMax.json", MotorControllerConfig.class)
                      ),
                      new AnalogInput(ANGLE_ENCODER_BACK_LEFT_PORT),
                      configUtils.readConfigFromClasspath("Swerve/BackLeft/ShiftingSwerveModule.json", ShiftingSwerveModuleConfig.class)
              )
      };
      return shiftingSwerveModules;
  }
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
