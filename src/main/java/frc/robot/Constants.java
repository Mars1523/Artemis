// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
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
    public static final String kLimelightCameraName = "limelight-shooter";
    // I dont know if we need this anymore because it can only hold 1 camera...
    public static final String kPhotonCameraName = "MyCamera";

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static double kPYController = 3;
        public static double kPXController = 3;
        public static double kPThetaController = 1;
        public static double kMaxSpeedMetersPerSecond = DriveConstants.kMaxVelocityMetersPerSecond * .50;
        public static double kMaxAccelerationMetersPerSecondSquared = 4;
        public static Constraints kThetaControllerConstraints =
                new Constraints(DriveConstants.kMaxAngularVelocityRadiansPerSecond * .5, (Math.PI * 2) / 2);
    }

    public static class Vision {
        private static final double camPitch = Units.degreesToRadians(0);
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, camPitch, 0));

        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        // Guy's Constants: (Status: Not Implemented)
        // To add additional cameras or change the Camera names, simply add aditional cameras.
        // I think its safe to remove all but one camera... I think...
        public static final String[] kPhotonCameraNames = {"Camera1", "Camera2"};

        public static final Transform3d kRobotToCamera1 =
                new Transform3d(new Translation3d(1, 1, 1), new Rotation3d(0, camPitch, 0));
        public static final Transform3d kRobotToCamera2 =
                new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, camPitch, 0));

        public static final Transform3d[] kRobotToCams = {kRobotToCamera1, kRobotToCamera2};

        public static final double maxPoseAmbiguity = 0.2;
        public static final double maxYawRate = Units.degreesToRadians(200);
    }

    public static final ModuleConfiguration ModuleType = SdsModuleConfigurations.MK4N_L2;

    public static void configMotor(SparkMax motor, boolean Inverted) {}

    public static final class DriveConstants {
        public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

        // Distance between left and right wheels
        public static final double kTrackWidthMeters = 0.5842 / 2;
        // Distance between front and back wheels
        public static final double kTrackBaseMeters = 0.6096 / 2;

        private static final Translation2d kFrontLeftLocation = new Translation2d(kTrackBaseMeters, kTrackWidthMeters);
        private static final Translation2d kFrontRightLocation =
                new Translation2d(kTrackBaseMeters, -kTrackWidthMeters);
        private static final Translation2d kBackLeftLocation = new Translation2d(-kTrackBaseMeters, kTrackWidthMeters);
        private static final Translation2d kBackRightLocation =
                new Translation2d(-kTrackBaseMeters, -kTrackWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        public static final double kNeoFreeSpinRpm = 5676;

        public static final double kMaxVelocityMetersPerSecond =
                (kNeoFreeSpinRpm / 60.0) * ModuleType.getDriveReduction() * ModuleType.getWheelDiameter() * Math.PI;

        public static final double kMaxAngularVelocityRadiansPerSecond =
                kMaxVelocityMetersPerSecond / Math.hypot(kTrackWidthMeters / 2, kTrackBaseMeters / 2) * .75;
    }

    public static final class ModuleConstants {

        public static final double MaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        // public static final int kEncoderCPR = 1024;
        // public static final double kWheelDiameterMeters = 0.15;
        // public static final double kDriveEncoderDistancePerPulse =
        // // Assumes the encoders are directly mounted on the wheel shafts
        // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        // public static final double kTurningEncoderDistancePerPulse =
        // // Assumes the encoders are on a 1:1 reduction with the module shaft.
        // (2 * Math.PI) / (double) kEncoderCPR;
        public static final double TurningEncoderDegreesPerPulse =
                Math.toDegrees(2. * Math.PI * ModuleType.getSteerReduction());

        // public static final double kPModuleTurningController = 1;

        // public static final double kPModuleDriveController = 1;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }
}
