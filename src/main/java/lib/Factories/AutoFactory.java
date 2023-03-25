package lib.Factories;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

public class AutoFactory {
    
    public static SwerveAutoBuilder swerveAutoBuilderFactory(ShiftingSwerveDrive mSwerveDrive, Map<String, Command> mEventMap){
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            mSwerveDrive::getPose, 
            mSwerveDrive::updatePose,
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0),
            mSwerveDrive::drive,
            mEventMap,
            true,
            mSwerveDrive);

        return autoBuilder;
    }
}
