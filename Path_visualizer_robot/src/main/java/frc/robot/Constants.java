// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class INT_PORT {
        public static final int rightmaster = 1;
        public static final int leftmaster = 2;
        public static final int leftfollow = 3;
        public static final int rightfollow = 4;
    }
    public static final class ROBOT_DATA {
        public static final double track_width = 456; // in meters
        public static final double left_motor_gear_ratio = 72.9;
        public static final double right_motor_gear_ratio = 100;
        public static final double wheel_radius = 3; // in meters

    }
}
