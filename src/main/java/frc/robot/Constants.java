// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */
public class Constants {
    public static final double METERS_PER_INCH = 0.0254; // Thanks hayden

    public static final class Pneumatics {
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SWERVE_FORWARD = 0;
        public static final int SWERVE_REVERSE = 0;
    }

    public static final class Swerve {
        public static final int SPEED_FRONT_RIGHT_PORT = 0;
        public static final int SPEED_FRONT_LEFT_PORT = 0;
        public static final int SPEED_BACK_RIGHT_PORT = 0;
        public static final int SPEED_BACK_LEFT_PORT = 0;

        public static final int ANGLE_FRONT_RIGHT_PORT = 0;
        public static final int ANGLE_FRONT_LEFT_PORT = 0;
        public static final int ANGLE_BACK_RIGHT_PORT = 0;
        public static final int ANGLE_BACK_LEFT_PORT = 0;

        public static final int ANGLE_ENCODER_FRONT_RIGHT_PORT = 0;
        public static final int ANGLE_ENCODER_FRONT_LEFT_PORT = 0;
        public static final int ANGLE_ENCODER_BACK_RIGHT_PORT = 0;
        public static final int ANGLE_ENCODER_BACK_LEFT_PORT = 0;

    }
}
