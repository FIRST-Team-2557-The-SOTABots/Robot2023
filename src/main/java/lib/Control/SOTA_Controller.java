// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Control;

/** Add your docs here. */
public interface SOTA_Controller {
    public boolean getStart();
    public boolean getBack();
    public boolean getA();
    public boolean getB();
    public boolean getX();
    public boolean getY();
    public boolean getLeftBumper();
    public boolean getRightBumper();
    public double getLeftX();
    public double getLeftY();
    public double getRightX();
    public double getRightY();
    public double getLeftTrigger();
    public double getRightTrigger();

    public default double deadband(double input) {
        return Math.abs(input < 0.1 ? 0 : input);
    }

    public default double square(double input) {
        return Math.signum(input) * input * input;
    }
}
