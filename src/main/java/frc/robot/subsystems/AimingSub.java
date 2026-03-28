package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AimingSub extends SubsystemBase {
    private static final Translation2d centerOffset = new Translation2d(-0.0762, 0.0635);
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private TurretSubsystem turretSubsystem;
    private LauncherSubsystem launcherSubsystem;

    // see here: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // values given in pdf are in inches, convert to meters
    // these are the "Welded" perimeter values (not AndyMark)

    // 158.32 inches
    private final double kArenaCenterY = 4.034663;
    private final double kArenaBlueTransitionX = 4.625594;
    private final double kArenaRedTransitionX = 11.915394;

    private Translation2d blueHub = new Translation2d(kArenaBlueTransitionX, kArenaCenterY);
    private Translation2d redHub = new Translation2d(kArenaRedTransitionX, kArenaCenterY);

    private Translation2d UphomeR = new Translation2d(11.4, 5.5);
    private Translation2d DownhomeR = new Translation2d(11.4, 2.5);
    private Translation2d UphomeB = new Translation2d(4, 5.5);
    private Translation2d DownhomeB = new Translation2d(4, 2.5);

    public Distance robotToHubDistancePhoton;
    public Rotation2d turretAnglePhoton;

    public AimingSub(
            SwerveDriveSubsystem swerveDriveSubsystem,
            TurretSubsystem turretSubsystem,
            LauncherSubsystem launcherSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.launcherSubsystem = launcherSubsystem;
    }

    public Time getTime(Distance distance) {
        double distanceInches = distance.abs(Inches);
        double a = 0.0143;
        double b = 1.56;
        double c = 790;
        // Quadradic R=0.989
        Time time = Milliseconds.of(a * Math.pow(distanceInches, 2) + b * Math.pow(distanceInches, 1) + c);
        return time;
    }

    public void setAngle(Rotation2d angle) {
        turretSubsystem.setTurretAngle(angle);
    }

    public Rotation2d getAngle() {
        return turretSubsystem.getTurretSetpoint();
    }

    public Command shootPhotonCommand() {
        return run(() -> {
                    Logger.recordOutput("Aiming/RunningShootPhotonCommand", true);
                    launcherSubsystem.shootDistance(this.robotToHubDistancePhoton);
                    turretSubsystem.setTurretAngle(this.turretAnglePhoton);
                    if (isAimingReady()) {
                        launcherSubsystem.runFeed();
                    } else {
                        launcherSubsystem.stopFeed();
                    }
                    swerveDriveSubsystem.setIsShooting(true);
                })
                .finallyDo(() -> {
                    launcherSubsystem.turnOff();
                    swerveDriveSubsystem.setIsShooting(false);
                });
    }

    public boolean isAimingReady() {
        boolean isLauncherReady = launcherSubsystem.isLauncherReady();
        boolean isTurretReady = turretSubsystem.isTurretReady();
        return isLauncherReady && isTurretReady;
    }

    public Translation2d getHubPos() {
        Translation2d hubPos = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? redHub : blueHub;
        return hubPos;
    }

    public Translation2d getUpHomePos() {
        Translation2d upHomePos = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? UphomeR : UphomeB;
        return upHomePos;
    }

    public Translation2d getDownHomePos() {
        Translation2d downHomePos =
                DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? DownhomeR : DownhomeB;
        return downHomePos;
    }

    @Override
    public void periodic() {
        Translation2d targetPosition;
        Rotation2d robotPoseAngle = swerveDriveSubsystem.getPose().getRotation();
        Translation2d robotFieldPosition = swerveDriveSubsystem.getPose().getTranslation();
        Translation2d turretFieldPosition = robotFieldPosition.plus(centerOffset.rotateBy(robotPoseAngle));

        if (turretFieldPosition.getX() < 4.63 || turretFieldPosition.getX() > 11.91) {
            targetPosition = getHubPos();
        } else {
            targetPosition = turretFieldPosition.getY() > 4.03 ? getUpHomePos() : getDownHomePos();
        }

        Translation2d robotToHub = targetPosition.minus(turretFieldPosition);
        double xV = swerveDriveSubsystem.getChassisSpeeds().vxMetersPerSecond;
        double yV = swerveDriveSubsystem.getChassisSpeeds().vyMetersPerSecond;
        Distance hubDistance = Meters.of(robotToHub.getNorm());
        Time time = getTime(hubDistance);

        for (int i = 0; i < 4; i++) {
            Translation2d compensation = new Translation2d(xV * time.abs(Seconds), yV * time.abs(Seconds));
            robotToHub = targetPosition.minus(turretFieldPosition).minus(compensation);
            hubDistance = Meters.of(robotToHub.getNorm());
            time = getTime(hubDistance);
        }

        // Rotation2d halfRotation = Math.PI;
        Rotation2d robotToHubFieldAngle = robotToHub.getAngle();
        Rotation2d robotPoseToHubAngle = robotToHubFieldAngle.minus(robotPoseAngle);
        Distance robotToHubDistance = Meters.of(robotToHub.getNorm());

        this.robotToHubDistancePhoton = robotToHubDistance;
        Logger.recordOutput("Aiming/robotPoseToHubAngle", robotPoseToHubAngle.getRotations());
        Logger.recordOutput(
                "Aiming/minusRobotPoseToHubAngle",
                robotPoseToHubAngle.unaryMinus().getRotations());
        Logger.recordOutput(
                "Aiming/turretAngle",
                Rotations.of(robotPoseToHubAngle.unaryMinus().getRotations()));

        this.turretAnglePhoton = robotPoseToHubAngle.unaryMinus();

        Logger.recordOutput("Aiming/robotToHopperFieldAngle", robotToHubFieldAngle);
        Logger.recordOutput("Aiming/robotPoseToHopperAngle", robotPoseToHubAngle);

        Time estimatedTime = getTime(Meters.of(robotToHub.getNorm()));
        Logger.recordOutput("Aiming/EstimatedTime", estimatedTime);
        Logger.recordOutput("Aiming/robotToHubDistance", robotToHubDistance);

        Logger.recordOutput("Aiming/IsAimingReady", isAimingReady());

        // record the actual turret angle here
        Rotation2d turretFieldAngle =
                robotPoseAngle.plus(turretSubsystem.getTurretAngle().unaryMinus());
        Pose2d turretPose2d = new Pose2d(turretFieldPosition, turretFieldAngle);
        Logger.recordOutput("Aiming/TurretPose2d", turretPose2d);

        Logger.recordOutput("Aiming/TargetPosition", targetPosition);
    }
}
