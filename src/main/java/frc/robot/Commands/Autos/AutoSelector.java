// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import lib.Command.AutoCommand;

/** Add your docs here. */
public class AutoSelector {
    private SendableChooser<AutoCommand> mAutoSelector;

    public AutoSelector(SendableChooser<AutoCommand> autoChooser) {
        mAutoSelector = autoChooser;
    }

    public void registerAuto(String name, AutoCommand auto) {
        mAutoSelector.addOption(name, auto);
    }

    public Pose2d getInitPose() {
        return mAutoSelector.getSelected().getInitPose();
    }
} 
