// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Config;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class DoubleSolenoidSwerveShifterConfig {
    private String hiGearValue;
    private String loGearValue;
    
    private Value convertStringToValue(String str) {
        switch(str) {
            case("FORWARD"): return Value.kForward;
            case("REVERSE"): return Value.kReverse;
            default: throw new IllegalArgumentException("Illegal value ");
        }
    }

    public Value getHiGearValue() {
        return convertStringToValue(hiGearValue);
    }

    public Value getLoGearValue() {
        return convertStringToValue(loGearValue);
    }
}
