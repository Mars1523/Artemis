package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
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
    Rotation2d robotToHopperFieldAngle;
    Rotation2d robotPoseToHopperAngle;
    Rotation2d robotPoseAngle;

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

    public Translation2d getCoordinates(Translation2d hubPosition) {
        robotPoseAngle = swerveDriveSubsystem.getPose().getRotation();
        var robotFieldPosition = swerveDriveSubsystem.getPose().getTranslation().plus(centerOffset.rotateBy(robotPoseAngle));

        Translation2d finalTranslation = hubPosition.minus(robotFieldPosition);
        double xV = swerveDriveSubsystem.getChassisSpeeds().vxMetersPerSecond;
        double yV = swerveDriveSubsystem.getChassisSpeeds().vyMetersPerSecond;
        Logger.recordOutput("Aiming/xV", xV);
        Logger.recordOutput("Aiming/yV", yV);
        Distance hubDistance = Meters.of(finalTranslation.getNorm());
        Time time = getTime(hubDistance);

        for (int i = 0; i < 4; i++) {
            Logger.recordOutput("Aiming/FinalTranslation" + i, finalTranslation);
            Translation2d compensation = new Translation2d(xV * time.abs(Seconds), yV * time.abs(Seconds));
            Logger.recordOutput("Aiming/Compensation" + i, compensation);
            Logger.recordOutput("Aiming/time" + i, time.abs(Seconds));
            finalTranslation = hubPosition.minus(compensation);
            hubDistance = Meters.of(finalTranslation.minus(robotFieldPosition).getNorm());
            time = getTime(hubDistance);

            robotToHopperFieldAngle = (finalTranslation.minus(robotFieldPosition)).getAngle();
            robotPoseToHopperAngle = robotToHopperFieldAngle.minus(robotPoseAngle);
        }

        return finalTranslation;
    }

    public Command shootCommand() {
        return run(() -> shoot()).finallyDo(() -> launcherSubsystem.setMotorDuty(0));
    }

    public void shoot() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubPose = alliance == Alliance.Red ? redHub : blueHub;
        Translation2d finalTranslation = getCoordinates(hubPose);
        Logger.recordOutput("Aiming/FinalTranslation", finalTranslation);
        Logger.recordOutput("Aiming/TurretAngle", finalTranslation.getAngle().unaryMinus());

        turretSubsystem.setTurretAngle(robotPoseToHopperAngle.unaryMinus());
        launcherSubsystem.shootDistance(finalTranslation.getNorm());

        Time estimatedTime = getTime(Meters.of(finalTranslation.getNorm()));
        Logger.recordOutput("Aiming/EstimatedTime", estimatedTime.abs(Seconds));
        Logger.recordOutput("Aiming/RobotFieldAngle", robotToHopperFieldAngle);
        Logger.recordOutput("Aiming/RobotHopperAngle", robotToHopperFieldAngle);
    }

    @Override
    public void periodic() {}
}
