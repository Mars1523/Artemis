package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AimingSubsystem extends SubsystemBase {

    // found using PathPlanner
    private final Translation2d kRedHubCoordinates = new Translation2d(11.919, 4.029);
    private final Translation2d kBlueHubCoordinates = new Translation2d(4.621, 4.029);

    private final Translation2d shooterOffsetFromRobotCenter = new Translation2d(-0.0762, .0635);

    SwerveDriveSubsystem swerveDriveSubsystem;
    LauncherSubsystem launcherSubsystem;
    TurretSubsystem turretSubsystem;

    public AimingSubsystem(
            SwerveDriveSubsystem swerveDriveSubsystem,
            LauncherSubsystem launcherSubsystem,
            TurretSubsystem turretSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    public Command shootAtHubCommand() {
        return run(() -> shootAtHub()).finallyDo(() -> launcherSubsystem.setMotorDuty(0));
    }

    public void shootAtHub() {
        Translation2d hubCoordinates =
                DriverStation.getAlliance().get() == Alliance.Red ? kRedHubCoordinates : kBlueHubCoordinates;

        Pose2d robotPose = swerveDriveSubsystem.getPose();
        Translation2d robotCoordinates = robotPose.getTranslation();
        Translation2d turretCoordinates =
                robotCoordinates.plus(shooterOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));
        Translation2d toAllianceHub = hubCoordinates.minus(turretCoordinates);

        Logger.recordOutput("Aiming/hubCoordinates", hubCoordinates);
        Logger.recordOutput("Aiming/turretCoordinates", turretCoordinates);
        Logger.recordOutput("Aiming/robotCoordinates", robotCoordinates);

        double distance = toAllianceHub.getNorm();
        double flightTimeSeconds = getTimeOfFlight(distance);
        ChassisSpeeds robotVelocity = swerveDriveSubsystem.getRobotVelocity();
        double xError = robotVelocity.vxMetersPerSecond * flightTimeSeconds;
        double yError = robotVelocity.vyMetersPerSecond * flightTimeSeconds;
        Translation2d error = new Translation2d(xError, yError);
        Translation2d correctedToAllianceHub = toAllianceHub.minus(error);

        double correctedDistance = correctedToAllianceHub.getNorm();
        launcherSubsystem.shootDistance(correctedDistance);

        Rotation2d robotAngleField = robotPose.getRotation();
        Rotation2d hubAngleField = correctedToAllianceHub.getAngle();
        Rotation2d hubAngleRobot = hubAngleField.minus(robotAngleField);
        turretSubsystem.setTurretAngle(hubAngleRobot);

        Logger.recordOutput("Aiming/robotAngleField", robotAngleField.getDegrees());
        Logger.recordOutput("Aiming/hubAngleField", hubAngleField.getDegrees());
        Logger.recordOutput("Aiming/hubAngleRobot", hubAngleRobot.getDegrees());
    }

    private double getTimeOfFlight(double distance) {
        // todo: measure values or use kinematics
        // currently a placeholder
        return 1.0;
    }
}
