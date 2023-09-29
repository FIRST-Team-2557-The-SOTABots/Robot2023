// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.Control;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/** Add your docs here. */
public class SOTA_PS4Controller extends CommandPS4Controller implements SOTA_Controller {

    private boolean mSquareInputs;

    public SOTA_PS4Controller(int port) {
        this(port, true);
    }

    public SOTA_PS4Controller(int port, boolean squareInputs) {
        super(port);
        this.mSquareInputs = squareInputs;
    }

    public boolean getStart() {
        return options().getAsBoolean();
    }

    public boolean getBack() {
        return share().getAsBoolean();
    }

    public boolean getA() {
        return super.cross().getAsBoolean();
    }

    public boolean getB() {
        return super.circle().getAsBoolean();
    }

    public boolean getX() {
        return super.square().getAsBoolean();
    }

    public boolean getY() {
        return super.triangle().getAsBoolean();
    }

    public boolean getLeftBumper() {
        return super.L1().getAsBoolean();
    }

    public boolean getRightBumper() {
        return super.R1().getAsBoolean();
    }

    public double getLeftX() {
        double input = deadband(super.getLeftX());
        if (mSquareInputs){
            input = square(input); 
        }
        return input;
    }

    public double getLeftY() {
        double input = deadband(super.getLeftY());
        if (mSquareInputs){
            input = square(input); 
        }
        return input;
    }

    public double getRightX() {
        double input = deadband(super.getRightX());
        if (mSquareInputs){
            input = square(input); 
        }
        return input;
    }

    public double getRightY() {
        double input = deadband(super.getRightY());
        if (mSquareInputs){
            input = square(input); 
        }
        return input;
    }

    public double getLeftTrigger() {
        double input = deadband(getL2Axis());
        if (mSquareInputs) {
            input = square(input);
        }
        return input;
    }

    public double getRightTrigger() {
        double input = deadband(getL2Axis());
        if (mSquareInputs) {
            input = square(input);
        }
        return input;
    }

}
