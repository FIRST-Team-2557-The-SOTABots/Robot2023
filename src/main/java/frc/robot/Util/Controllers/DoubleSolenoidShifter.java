package frc.robot.Util.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Util.Configs.DoubleSolenoidConfig;
import frc.robot.Util.Interfaces.GearShifter;

public class DoubleSolenoidShifter implements GearShifter{
    
    private DoubleSolenoid shifter;
    int[] gearRatios;

    private Value kHiGearValue;
    private Value kLoGearValue;

    public DoubleSolenoidShifter(DoubleSolenoid shifter, DoubleSolenoidConfig config) {
        this.shifter = shifter; gearRatios = config.getGearRatios();
    }

    @Override
    public int getGear() {
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
