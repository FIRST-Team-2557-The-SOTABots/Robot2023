package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Rotation;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;
import lib.Command.AutoCommand;

public class PlaceConeAndMobility extends SequentialCommandGroup implements AutoCommand{
    private Extension mExtension;
    private Intake mIntake;
    private Rotation mRotation;
    private ShiftingSwerveDrive mSwerveDrive;

    public PlaceConeAndMobility(Extension mExtension,
    Intake mIntake,
    Rotation mRotation,
    ShiftingSwerveDrive mSwerveDrive
    ){
          this.mExtension = mExtension;
      this.mIntake = mIntake;
      this.mRotation = mRotation;
      this.mSwerveDrive = mSwerveDrive;
        addRequirements(mExtension, mIntake, mRotation, mSwerveDrive);

    }

    

    @Override
    public Pose2d getInitPose() {
        // TODO Auto-generated method stub
        return new Pose2d();
    }
    
}
