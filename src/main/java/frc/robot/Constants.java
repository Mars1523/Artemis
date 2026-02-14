// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.sds.ModuleConfiguration;
import frc.robot.sds.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    /**
     * These are copied from Mariner
     * Needs to be updated for 2026 bot swerve drive
     */
    public static class SwerveDriveConstants {
        public static ModuleConfiguration kSwerveModuleConfiguration = SdsModuleConfigurations.MK4_L4;

        // Distance between left and right wheels
        public static final double kTrackWidthMeters = 0.5842 / 2;
        // Distance between front and back wheels
        public static final double kTrackBaseMeters = 0.6096 / 2;

        // This max speed was theoretically based on free spin rpm,
        // but could be improved by measuring actual max driving speed in practice
        public static final double kNeoFreeSpinRpm = 5676;
        public static final double kMaxVelocityMetersPerSecond = (kNeoFreeSpinRpm / 60.0)
                * kSwerveModuleConfiguration.getDriveReduction()
                * kSwerveModuleConfiguration.getWheelDiameter()
                * Math.PI;
        public static final double kMaxAngularVelocityRadiansPerSecond =
                kMaxVelocityMetersPerSecond / Math.hypot(kTrackWidthMeters / 2, kTrackBaseMeters / 2) * .75;
    }
}
