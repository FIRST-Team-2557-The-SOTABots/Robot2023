package frc.robot.Commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class PlaceCondAndMobilityWithPath extends SequentialCommandGroup{
    private ExtensionPID mExtensionCommand;
    private RotationPID mRotationPID;
    private SwerveAutoBuilder autoBuilder;
    private PathPlannerTrajectory trajectory;
    private Intake mIntake;

    public PlaceCondAndMobilityWithPath(
        ShiftingSwerveDrive mSwerveDrive,
        ExtensionPID mExtensionCommand,
        RotationPID mRotationPID,
        SwerveAutoBuilder autoBuilder,
        Intake mIntake,
        PathPlannerTrajectory trajectory){
            

        this.mExtensionCommand = mExtensionCommand;
        this.mRotationPID = mRotationPID;
        this.autoBuilder = autoBuilder;
        this.trajectory = trajectory;
        this.mIntake = mIntake;
        addRequirements(mSwerveDrive);
        addCommands(
            new InstantCommand(() -> {
                mSwerveDrive.updatePose(trajectory.getInitialState());
                mSwerveDrive.shift(0);
            }),
            new ParallelCommandGroup(
            mExtensionCommand,
            mRotationPID,
            new SequentialCommandGroup(
                    new InstantCommand(() ->{
                        mRotationPID.setSetpoint(RotationSetpoint.HIGH);
                        mExtensionCommand.setSetpoint(ExtensionSetpoint.HIGH);
                    }),
                    new WaitUntilCommand(mRotationPID::atSetpoint),
                    new WaitUntilCommand(mExtensionCommand::atSetpoint),
                    new InstantCommand(mIntake::outTakeCone, mIntake),
                    new WaitUntilCommand(1),
                    new InstantCommand(mIntake::stop),
                    new InstantCommand(() -> {
                        mRotationPID.setSetpoint(RotationSetpoint.RESET);
                        mExtensionCommand.setSetpoint(ExtensionSetpoint.RESET);
                    }),
                    new WaitUntilCommand(mRotationPID::atSetpoint),
                    new WaitUntilCommand(mExtensionCommand::atSetpoint),
                    autoBuilder.followPath(trajectory).withTimeout(6.17)
                )
            )
        );
    }

    
    
}
