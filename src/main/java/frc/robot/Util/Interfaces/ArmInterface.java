package frc.robot.Util.Interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmInterface extends Subsystem{
    public void setPose();
    public Pose3d getPose3d();
    public void setAngle();
    public void setLength();
    public Rotation2d getAngle();
    public Rotation2d getRelativeRotation();
}
