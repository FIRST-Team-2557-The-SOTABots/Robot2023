package frc.robot.util.Subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeInterface extends Subsystem{
    public void intake();
    public void intakeCube();
    public void intakeCone();
    public boolean hasPiece();
    public void set(double speed);
    public void stop();
}
