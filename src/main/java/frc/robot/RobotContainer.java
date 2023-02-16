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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Gyro.NavX;
import frc.robot.Subsystems.Swerve.DoubleSolenoidSwerveShifter;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import frc.robot.Subsystems.Swerve.ShiftingSwerveModule;
import lib.Config.DoubleSolenoidSwerveShifterConfig;
import lib.Config.MotorControllerConfig;
import lib.Config.ShiftingSwerveDriveConfig;
import lib.Config.ShiftingSwerveModuleConfig;
import lib.MotorController.FalconDelegate;
import lib.MotorController.SparkMaxDelegate;

import static frc.robot.Constants.Pneumatics.*;
import static frc.robot.Constants.Swerve.*;

public class RobotContainer {

  ShiftingSwerveDrive mShiftingSwerveDrive;
  DoubleSolenoidSwerveShifter mShifter;
  NavX mNavx;

  public RobotContainer() throws JsonParseException, JsonMappingException, IOException {
    ObjectMapper mapper = new ObjectMapper();
    mapper.disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);

    // TODO: Shorten these lines of swerve module declaration
    ShiftingSwerveModule[] shiftingSwerveModules = {
      // Front Right
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_FRONT_RIGHT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontRight\\SpeedFalcon.json"), 
            MotorControllerConfig.class
          )
        ),
        new SparkMaxDelegate(
          ANGLE_FRONT_RIGHT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontRight\\AngleSparkMax.json"), 
            MotorControllerConfig.class
          )
        ),
        new AnalogInput(ANGLE_ENCODER_FRONT_RIGHT_PORT),
        mapper.readValue(
          RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontRight\\ShiftingSwerveModule.json"), 
          ShiftingSwerveModuleConfig.class
        )
      ),
      // Front Left
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_FRONT_LEFT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontLeft\\SpeedFalcon.json"), 
            MotorControllerConfig.class
          )
        ),
        new SparkMaxDelegate(
          ANGLE_FRONT_LEFT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontLeft\\AngleSparkMax.json"), 
            MotorControllerConfig.class
          )
        ),
        new AnalogInput(ANGLE_ENCODER_FRONT_LEFT_PORT),
        mapper.readValue(
          RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\FrontLeft\\ShiftingSwerveModule.json"), 
          ShiftingSwerveModuleConfig.class
        )
      ),
      // Back Right
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_BACK_RIGHT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackRight\\SpeedFalcon.json"), 
            MotorControllerConfig.class
          )
        ),
        new SparkMaxDelegate(
          ANGLE_BACK_RIGHT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackRight\\AngleSparkMax.json"), 
            MotorControllerConfig.class
          )
        ),
        new AnalogInput(ANGLE_ENCODER_BACK_RIGHT_PORT),
        mapper.readValue(
          RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackRight\\ShiftingSwerveModule.json"), 
          ShiftingSwerveModuleConfig.class
        )
      ),
      // Back Left
      new ShiftingSwerveModule(
        new FalconDelegate(
          SPEED_BACK_LEFT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackLeft\\SpeedFalcon.json"), 
            MotorControllerConfig.class
          )
        ),
        new SparkMaxDelegate(
          ANGLE_BACK_LEFT_PORT, 
          mapper.readValue(
            RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackLeft\\AngleSparkMax.json"), 
            MotorControllerConfig.class
          )
        ),
        new AnalogInput(ANGLE_ENCODER_BACK_LEFT_PORT),
        mapper.readValue(
          RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\BackLeft\\ShiftingSwerveModule.json"), 
          ShiftingSwerveModuleConfig.class
        )
      )
    };
    DoubleSolenoidSwerveShifter shifter = 
      new DoubleSolenoidSwerveShifter(
        new DoubleSolenoid(MODULE_TYPE, SWERVE_FORWARD, SWERVE_REVERSE),
        mapper.readValue(
          RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\DoubleSolenoidSwerveShifter"),
          DoubleSolenoidSwerveShifterConfig.class
        )
      );
    NavX navX = new NavX(new AHRS(Port.kMXP));
    mShiftingSwerveDrive = new ShiftingSwerveDrive(
      shiftingSwerveModules, 
      shifter, 
      navX, 
      mapper.readValue(
        RobotContainer.class.getClassLoader().getResourceAsStream("Swerve\\ShiftingSwerveDrive.json"),
        ShiftingSwerveDriveConfig.class
      )
    );

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
