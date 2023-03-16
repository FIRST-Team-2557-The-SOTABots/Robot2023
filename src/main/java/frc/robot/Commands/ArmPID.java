// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.SuperStructure;
import lib.Gyro.SOTAGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends PIDCommand {
  /** Creates a new ArmPID. */
  public ArmPID(PIDController armPID, SuperStructure m_ArmSubsystem, SOTAGyro armGyro, double setPoint) {
    super(
        // The controller that the command will use
        armPID,
        // This should return the measurement
        () -> armGyro.getRoll(),
        // This should return the setpoint (can also be a constant)
        () -> setPoint,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("Arm PID Output:", output);
          // Use the output here
          m_ArmSubsystem.setRotatorSpeed(MathUtil.clamp(output, -0.5, 0.5));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_ArmSubsystem);
  }

  public void setSetpoint(){
    
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
