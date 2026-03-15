package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

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
    private static final Translation2d centerOffset = new Translation2d(0.0762, 0.0635);
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private TurretSubsystem turretSubsystem;
    private LauncherSubsystem launcherSubsystem;

    private Translation2d blueHub = new Translation2d(4.621, 4.016);
    private Translation2d redHub = new Translation2d(11.945, 3.990);

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
        return turretSubsystem.getTurretAngle();
    }

    public Command pointTurretPhotonCommand() {
        return run(() -> {
            turretSubsystem.setTurretAngle(this.turretAnglePhoton);
        });
    }

    public Command shootPhotonCommand() {
        return run(() -> launcherSubsystem.shootDistance(robotToHubDistancePhoton))
                .finallyDo(() -> launcherSubsystem.setMotorDuty(0));
    }

    @Override
    public void periodic() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubPosition = alliance == Alliance.Red ? redHub : blueHub;

        Rotation2d robotPoseAngle = swerveDriveSubsystem.getPose().getRotation();
        var robotFieldPosition =
                swerveDriveSubsystem.getPose().getTranslation().plus(centerOffset.rotateBy(robotPoseAngle));

        Translation2d robotToHub = hubPosition.minus(robotFieldPosition);
        double xV = swerveDriveSubsystem.getChassisSpeeds().vxMetersPerSecond;
        double yV = swerveDriveSubsystem.getChassisSpeeds().vyMetersPerSecond;
        Distance hubDistance = Meters.of(robotToHub.getNorm());
        Time time = getTime(hubDistance);

        for (int i = 0; i < 4; i++) {
            Translation2d compensation = new Translation2d(xV * time.abs(Seconds), yV * time.abs(Seconds));
            robotToHub = hubPosition.minus(robotFieldPosition).minus(compensation);
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
    }
}
