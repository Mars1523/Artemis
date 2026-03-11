package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AimingSubsystem extends SubsystemBase {
    public static class ShootingSolution {
        public boolean hasSolution = false;
        public double rangeMeters = 0.0;
        public double shooterRps = 0.0;
        public double ballSpeedMetersPerSecond = 0.0;
        public double flightTimeSeconds = 0.0;
        public double uncompensatedTurretYawDeg = 0.0;
        public double compensatedTurretYawDeg = 0.0;
        public double yawFeedforwardDegPerSec = 0.0;
        public double rangeRateMetersPerSec = 0.0;
        public Pose2d robotPose = new Pose2d();
        public Rotation2d compensatedTurretYawRot;
            }
            private final double shooterWheelRadiusMeters = 0.1016;
            private final double ballVelocityFactor = 0.8;
            private final double launchDelaySeconds;
            private final Translation2d shooterOffsetFromRobotCenter;
            SwerveDriveSubsystem swerveDriveSubsystem;
            LauncherSubsystem launcherSubsystem;
            public AimingSubsystem(
                    SwerveDriveSubsystem swerveDriveSubsystem,
                    double launchDelaySeconds,
                    Translation2d shooterOffsetFromRobotCenter,
                    LauncherSubsystem launcherSubsystem
                    ) {
                this.launchDelaySeconds = launchDelaySeconds;
                this.shooterOffsetFromRobotCenter = shooterOffsetFromRobotCenter;
                this.launcherSubsystem = launcherSubsystem;
                this.swerveDriveSubsystem = swerveDriveSubsystem;
            }
        
        
            /**
             * @param targetFieldPosition fixed field position of the target
             * @param fieldRelativeSpeeds robot field relative chassis speeds
             * @param currentTurretAngleRelativeToRobot turret angle relative to robot forward
             */
            
             
            public ShootingSolution getShootingSolution(
                    Translation2d targetFieldPosition,
                    ChassisSpeeds fieldRelativeSpeeds,
                    Rotation2d currentTurretAngleRelativeToRobot) {
        
                ShootingSolution solution = new ShootingSolution();
                Pose2d robotPose = swerveDriveSubsystem.getPose();
                solution.robotPose = robotPose;
                Translation2d shooterFieldPosition =
                        robotPose.getTranslation().plus(
                               shooterOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));
                Translation2d toAllianceHub = targetFieldPosition.minus(shooterFieldPosition);
        
                double rangeMeters = toAllianceHub.getNorm();
                if (rangeMeters < 1e-6) {
                    return solution;
                }
        
                solution.rangeMeters = rangeMeters;
                double shooterRps=LauncherSubsystem.launcherRpsForDistance(rangeMeters);
                solution.shooterRps = shooterRps;
                double ballSpeedMetersPerSecond =
                       2.0 * Math.PI * shooterWheelRadiusMeters * shooterRps * ballVelocityFactor; //velocity factor can be given by mechanical
                solution.ballSpeedMetersPerSecond = ballSpeedMetersPerSecond;
        
                if (ballSpeedMetersPerSecond < 1e-6) {
                    return solution; //avoid divide by zero error
                }
        
        
                double flightTimeSeconds = rangeMeters / ballSpeedMetersPerSecond;
                solution.flightTimeSeconds = flightTimeSeconds; //est ball flight time based on distance and speed
                Rotation2d lineOfSightField = toAllianceHub.getAngle();
        
                Rotation2d currentTurretFieldHeading = //converts turret heading into field coordinates
                        robotPose.getRotation().plus(currentTurretAngleRelativeToRobot);
        
                Rotation2d uncompensatedError = lineOfSightField.minus(currentTurretFieldHeading);

                solution.uncompensatedTurretYawDeg =
                        currentTurretAngleRelativeToRobot.getDegrees() + uncompensatedError.getDegrees();
        
                //Calculus
                double vx = fieldRelativeSpeeds.vxMetersPerSecond;
                double vy = fieldRelativeSpeeds.vyMetersPerSecond;
                double omega = fieldRelativeSpeeds.omegaRadiansPerSecond; //Angular velocity
                double cos = lineOfSightField.getCos();
                double sin = lineOfSightField.getSin();
        
                double radialComponent = vx * cos + vy * sin; //Overshoot/Undershoot
                double tangentialComponent = -vx * sin + vy * cos;//Left/Right
        
                solution.rangeRateMetersPerSec = -radialComponent;
        
                double flightLeadYawRad =
                        Math.atan2(-tangentialComponent, ballSpeedMetersPerSecond);
                double delayLeadYawRad = 0.0;
        
                if (launchDelaySeconds > 0.0) {
                    double tangentialShiftBeforeLaunch = tangentialComponent * launchDelaySeconds;
                    delayLeadYawRad = Math.atan2(-tangentialShiftBeforeLaunch, rangeMeters);
                }
        
                double totalLeadYawRad = flightLeadYawRad + delayLeadYawRad;
                Rotation2d compensatedTurretFieldHeading =
                        lineOfSightField.plus(Rotation2d.fromRadians(totalLeadYawRad));
        
                Rotation2d compensatedTurretRelativeToRobot =
                        compensatedTurretFieldHeading.minus(robotPose.getRotation());
        
                solution.compensatedTurretYawDeg = //if robot was standing still. returns angle
                        compensatedTurretRelativeToRobot.getDegrees();
                solution.compensatedTurretYawRot = Rotation2d.fromRotations(solution.compensatedTurretYawDeg);
            solution.yawFeedforwardDegPerSec =
                -Math.toDegrees(omega + tangentialComponent / rangeMeters);
        solution.hasSolution = true;
        return solution;
    }
}

