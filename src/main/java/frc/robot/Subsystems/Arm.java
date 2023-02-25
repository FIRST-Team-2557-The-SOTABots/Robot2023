package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Util.Interfaces.ArmInterface;
import frc.robot.Util.Interfaces.SOTAGyro;
import frc.robot.Util.Interfaces.SOTAMotorController;

public class Arm implements ArmInterface{
    SOTAMotorController angleMotor;
    SOTAMotorController extendMotor;
    SOTAGyro gyro;
    public Arm(SOTAMotorController angleMotor, SOTAMotorController extendMotor, SOTAGyro gyro){
        this.angleMotor = angleMotor; this.extendMotor = extendMotor; this.gyro = gyro;
    }
    @Override
    public void setPose() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public Pose3d getPose3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setAngle() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setLength() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation2d getRelativeRotation() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
