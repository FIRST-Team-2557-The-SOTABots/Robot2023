package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ShiftingSwerveModuleState extends SwerveModuleState {

    private int gear;
    

    public ShiftingSwerveModuleState(int gear){
        this(new SwerveModuleState(), gear);
    }

    public ShiftingSwerveModuleState(int speed, Rotation2d angle, int gear){
        this(new SwerveModuleState  (speed, angle), gear);
    }

    public ShiftingSwerveModuleState(SwerveModuleState state, int gear){
        super(state.speedMetersPerSecond, state.angle);
        this.gear = gear;
    }
    public int getGear(){
        return gear;
    }
    public static ShiftingSwerveModuleState[] toShiftingSwerveModuleState(SwerveModuleState[] state, int gear) {
        ShiftingSwerveModuleState[] shiftingStates = new ShiftingSwerveModuleState[state.length];
        for (int i = 0; i < state.length; i++) {
            shiftingStates[i] = new ShiftingSwerveModuleState(state[i], gear);
        }
        return shiftingStates;
    }
    
    public static ShiftingSwerveModuleState optimize(ShiftingSwerveModuleState state, Rotation2d rotation){
        return new ShiftingSwerveModuleState(SwerveModuleState.optimize(state, rotation), state.gear);
    }
    
}
