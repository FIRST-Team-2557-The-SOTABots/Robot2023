package frc.robot.Commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ExtensionPID;
import frc.robot.Commands.RotationPID;
import frc.robot.Commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.Commands.RotationPID.RotationSetpoint;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class PlaceConeAndMobility extends SequentialCommandGroup {

    private static PathPlannerTrajectory kMobility = PathPlanner.loadPath("MobilityFrom", 4, 3.5);
    private static double kExtendTimeout = 1.5;
    private static double kPlaceConeTimeout = 0.5;

    private Extension mExtension;
    private ExtensionPID mExtensionPID;
    private Rotation mRotation;
    private RotationPID mRotationPID;
    private Intake mIntake;
    private ShiftingSwerveDrive mSwerveDrive;
    private SwerveAutoBuilder mAutoBuilder;

    public PlaceConeAndMobility(Extension extension, 
        ExtensionPID extensionPID, 
        Rotation rotation, 
        RotationPID rotationPID, 
        Intake intake, 
        ShiftingSwerveDrive swerveDrive, 
        SwerveAutoBuilder autoBuilder) {
        this.mExtension = extension;
        this.mExtensionPID = extensionPID;
        this.mRotation = rotation;
        this.mRotationPID = rotationPID;
        this.mIntake = intake;
        this.mSwerveDrive = swerveDrive;
        addRequirements(mExtension, intake, rotation, swerveDrive);
        addCommands(
            new InstantCommand(
                () -> {
                    mRotationPID.setSetpoint(RotationSetpoint.HIGH);
                    mExtensionPID.setSetpoint(ExtensionSetpoint.HIGH);
                    mSwerveDrive.updatePose(kMobility.getInitialState());
                    mSwerveDrive.shift(0);
                }, mExtension, mRotation, mSwerveDrive
            ),
            new WaitCommand(kExtendTimeout),
            new RunCommand(
                () -> {
                    mIntake.set(-1); // TODO: Double check this 
                }, intake
            ).withTimeout(kPlaceConeTimeout),
            mAutoBuilder.followPath(kMobility)
        );

    }
    
}
