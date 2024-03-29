package lib.Factories;

import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class AutoFactory {
    
    public static SwerveAutoBuilder swerveAutoBuilderGenerator(ShiftingSwerveDrive mSwerveDrive, Map<String, Command> mEventMap){
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            mSwerveDrive::getPose, 
            mSwerveDrive::updatePose,
            new PIDConstants(2, 0, 0),//TODO: Don't hardcode!!!
            new PIDConstants(2, 0, 0),
            mSwerveDrive::drive,
            mEventMap,
            true,
            mSwerveDrive);

        return autoBuilder;
    }
}
