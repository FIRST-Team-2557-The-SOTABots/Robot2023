// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */
public class Constants {
    public static final double METERS_PER_INCH = 0.0254; // Thanks hayden

    public static final class Pneumatics {
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final int SWERVE_FORWARD = 0;
        public static final int SWERVE_REVERSE = 1;
    }

    public static final class Swerve {
        public static final int SPEED_FRONT_RIGHT_PORT = 6;
        public static final int SPEED_FRONT_LEFT_PORT = 10;
        public static final int SPEED_BACK_RIGHT_PORT = 2;
        public static final int SPEED_BACK_LEFT_PORT = 13;

        public static final int ANGLE_FRONT_RIGHT_PORT = 5;
        public static final int ANGLE_FRONT_LEFT_PORT = 11;
        public static final int ANGLE_BACK_RIGHT_PORT = 3;
        public static final int ANGLE_BACK_LEFT_PORT = 12;

        public static final int ANGLE_ENCODER_FRONT_RIGHT_PORT = 3;
        public static final int ANGLE_ENCODER_FRONT_LEFT_PORT = 0;
        public static final int ANGLE_ENCODER_BACK_RIGHT_PORT = 2;
        public static final int ANGLE_ENCODER_BACK_LEFT_PORT = 1;

    }
}
