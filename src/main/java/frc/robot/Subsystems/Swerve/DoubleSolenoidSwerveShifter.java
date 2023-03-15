// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import lib.Config.DoubleSolenoidConfig;
import lib.Pneumatics.GearShifter;

/** Add your docs here. */
public class DoubleSolenoidSwerveShifter implements GearShifter {
    private DoubleSolenoid mShifter;

    private Value kHiGearValue;
    private Value kLoGearValue;

    public DoubleSolenoidSwerveShifter(DoubleSolenoid shifter, DoubleSolenoidConfig config) {
        mShifter = shifter;
        kHiGearValue = config.getHiGearValue();
        kLoGearValue = config.getLoGearValue();
    }

    public void shift(int gear) {
        MathUtil.clamp(gear, 0, 1);
        mShifter.set(gear == 0 ? kLoGearValue : kHiGearValue);        
    }

    public int getGear() {
        return mShifter.get() == kLoGearValue ? 0 : 1;
    }

}
