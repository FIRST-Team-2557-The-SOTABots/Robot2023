// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

//TODO: You could extract this out into an "InterpolatingTreeMap" however that takes time that I do not have
/** Add your docs here. */
public class InterpolatingSwerveOffsetTreeMap extends TreeMap<Double, Double> {
    private double kNativeCountsPerRevolution;
    private double kAdjustedCountsPerRevolution;

    public InterpolatingSwerveOffsetTreeMap(Map<Double, Double> map, double nativeCountsPerRevolution) {
        super(map);
        kNativeCountsPerRevolution = nativeCountsPerRevolution;
        kAdjustedCountsPerRevolution = getAdjustedCountsPerRevolution();
    }

    /**
     * Gets the initial offset
     * @return initial offset
     */
    private double getOffset() {
        return super.firstKey();
    }

    /**
     * Gets the last value 
     * Name is shortened version of "Value Of N"
     * @return last value
     */
    private double getValueN() {
        return super.lastKey();
    }

    /**
     * Gets the adjusted encoder tick value gived radians, does not need to be in map
     * @param radians desired radians
     * @return adjusted encoder ticks
     */
    public double getInterpolated(double radians) {
        if (super.containsKey(radians)) return get((radians));
        // Silly basic liner interpolation
        double floorKey = super.floorKey(radians);
        double ceilingKey = super.ceilingKey(radians);
        double dy = super.get(ceilingKey) - super.get(floorKey);
        double dx = ceilingKey - floorKey;
        return (dy/dx) * radians + getOffset();
    }

    /**
     * Converts native encoder to adjusted range and offset
     * @param nat Native encoder ticks
     * @return Adjusted encoder ticks
     */
    public double nativeToAdjusted(double nat) {
        double deltaNative = ((kNativeCountsPerRevolution - getOffset()) + nat) % kNativeCountsPerRevolution;
        // Get the ratio between delta native and delta positive and 
        return (deltaNative / kNativeCountsPerRevolution) * kAdjustedCountsPerRevolution; 
    }

    /**
     * Get the adjusted counts per revolution with the first and last points of the map
     * @return Adjusted counts per revolution 
     */
    public double getAdjustedCountsPerRevolution() {
        double offsetToEnd = (kNativeCountsPerRevolution - getOffset());
        // Gets the absolute from offset to the right edge (kNativeCountsPerRevolution) and the left edge (zero) to valueN 
        // and wraps it around kNativeCountsPerRevolution 
        return -1 * (((offsetToEnd + getValueN()) - getOffset()) % kNativeCountsPerRevolution) + kNativeCountsPerRevolution;
    }
}
