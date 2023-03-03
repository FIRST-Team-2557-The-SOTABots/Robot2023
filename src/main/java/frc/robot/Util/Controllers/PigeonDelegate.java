package frc.robot.Util.Controllers;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Util.Interfaces.SOTAGyro;

public class PigeonDelegate extends WPI_Pigeon2 implements SOTAGyro{

    public PigeonDelegate(int deviceNumber) {
        super(deviceNumber);
    }

    public PigeonDelegate(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    @Override
    public void setAngle(double angle) {
        throw new IllegalAccessError("Cannot Set Angle on Pigeon");
        
    }

    @Override
    public void setAngle(Rotation2d angle) {
        throw new IllegalAccessError("Cannot Set Angle on Pigeon");
    }

    @Override
    public void resetAngle() {
        super.reset();
        
    }
    
}
