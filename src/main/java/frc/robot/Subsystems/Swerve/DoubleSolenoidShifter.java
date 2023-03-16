package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import lib.Config.DoubleSolenoidConfig;
import lib.Pneumatics.GearShifter;

public class DoubleSolenoidShifter implements GearShifter{
    
    private DoubleSolenoid shifter;
    private int[] gearRatios;

    private Value kHiGearValue;
    private Value kLoGearValue;

    public DoubleSolenoidShifter(DoubleSolenoid solenoid, DoubleSolenoidConfig config) {
        this.shifter = solenoid; gearRatios = config.getGearRatios(); this.kHiGearValue = Value.kForward; 
        this.kLoGearValue = Value.kReverse;
    }

    @Override
    public int getGear() {
        if(shifter == null) throw new RuntimeException("Null swerve module");
        return shifter.get() == kHiGearValue ? 1 : 0;
    }

    @Override
    public void shift(int gear) {
        MathUtil.clamp(gear, 0, 1);
        
        shifter.set(gear == 0 ? kLoGearValue : kHiGearValue); 
    }

    @Override
    public int shiftUp() {
        shifter.set(kHiGearValue);
        return getGear();
    }

    @Override
    public int shiftDown() {
        shifter.set(kLoGearValue);
        return getGear();
    }

    @Override
    public double getCurrentGearRatio() {
        
        return gearRatios[getGear()];
    }

    @Override
    public double getRatioFromGear(int gear) {
        return gearRatios[gear];
    }
    
}
