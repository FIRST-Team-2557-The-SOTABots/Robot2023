package lib.Configs;

import lib.MotorController.MotorLimits;

public class MotorLimitsConfig {
    private double upperLimit;
    private double lowerLimit;
    private boolean finalLimits;

    public double getUpperLimit(){
        return upperLimit;
    }

    public double getLowerLimit(){
        return lowerLimit;
    }

    public boolean getFinalLimits(){
        return finalLimits;
    }
    
    public MotorLimits getMotorLimits(){
        return new MotorLimits(lowerLimit, upperLimit, finalLimits);
    }
}
