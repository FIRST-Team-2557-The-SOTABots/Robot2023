package frc.robot.util.Gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends WPI_Pigeon2 implements SOTAGyro{

    public Pigeon(int deviceNumber) {
        super(deviceNumber);
    }

    public Pigeon(int deviceNumber, String canbus) {
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
