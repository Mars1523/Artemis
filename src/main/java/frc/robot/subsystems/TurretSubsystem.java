package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    // private Servo turret = new Servo(1);
    private SwerveDriveSubsystem swerve;
    private SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);
    SparkClosedLoopController turretController;

    SparkMaxConfig turretConfig = new SparkMaxConfig();

    public static double scale(double value, double minValue, double maxValue, double minOut, double maxOut) {
        return minOut + ((value - minValue) / (maxValue - minValue)) * (maxOut - minOut);
    }

    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(30, 10));
    TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();

    public TurretSubsystem(
            SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
        // setServoAngle(new Rotation2d(0))

        turretConfig.absoluteEncoder.zeroCentered(true);
        turretConfig.absoluteEncoder.inverted(true);
        turretConfig.absoluteEncoder.positionConversionFactor(1.5);
        turretController = turretMotor.getClosedLoopController();
        turretConfig.closedLoop.p(0).i(0).d(0);
        turretConfig.closedLoop.feedForward.kS(0).kV(0).kA(0);
        turretConfig.closedLoop.maxMotion.cruiseVelocity(0).maxAcceleration(0);
        turretConfig.smartCurrentLimit(40, 40);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretMotor.getEncoder().setPosition(turretMotor.getAbsoluteEncoder().getPosition());
    }

    // private void setServoAngle(Rotation2d angle) {
    //     var turretPercent = scale(angle.getDegrees(), -135, 135, 0, 1);
    //     turret.set(turretPercent);
    //     Logger.recordOutput("Turret/servoAngleDeg", angle.getDegrees());
    //     Logger.recordOutput("Turret/servoTurnPercent", turretPercent);
    // }

    // private void checkAlliance() {
    // if (isRed()) {
    // targetHopper = redHopper;
    // DownHome = new Translation2d(11.4, 2.5);
    // UpHome = new Translation2d(11.4, 5.5);
    // } else {
    // targetHopper = blueHopper;
    // DownHome = new Translation2d(4, 5.5);
    // UpHome = new Translation2d(4, 2.5);
    // }
    // }

    private Translation2d blueHopper = new Translation2d(4.621, 4.016);
    private Translation2d redHopper = new Translation2d(11.945, 3.990);

    Translation2d targetHopper() {
        return isRed() ? redHopper : blueHopper;
    }

    // TODO: Make functions
    // TODO: Make names better
    // Translation2d UpHome;
    // Translation2d DownHome;
    // Translation2d target;

    public Command shootAtHomeCommand() {
        return run(() -> shootAtHome());
    }

    public Command shootAtHopperCommand() {
        return run(() -> shootAtHopper());
    }

    public double motorToEncoder() {
        double encoderRotation = turretMotor.getAbsoluteEncoder().getPosition();
        return encoderRotation * (3 / 2) * 360;
    }

    double Turretsetpoint;

    public void setTurretSetpoint(Rotation2d angle) {
        Turretsetpoint = angle.getRotations();
    }

    /*
    public void shootAtHopperPhoton() {
        // Hopper aiming for testing. Photon.
        var result = photon.getLastResult();
        PhotonTrackedTarget target = result.getBestTarget();
        if (!result.hasTargets() || target == null) {
                turretMotor.set(0);
        *+ } else {
            Transform3d bestCameratoTarget = target.getBestCameraToTarget();
            double angle = bestCameratoTarget.getRotation().getZ();
            setTurretSetpoint(angle / (2*Math.PI));
        }
    }*/

    public void shootAtHopper() {
        var robotFieldPosition = swerve.getPose().getTranslation();
        if (robotFieldPosition.getX() == 0 && robotFieldPosition.getY() == 0) return;
        var robotPoseAngle = swerve.getPose().getRotation();
        var robotToHopperFieldAngle = targetHopper().minus(robotFieldPosition).getAngle();
        var robotPoseToHopperAngle = robotToHopperFieldAngle.minus(robotPoseAngle);

        Logger.recordOutput("Turret/BlueHopperPosition", blueHopper);
        Logger.recordOutput("Turret/RedHopperPosiiton", redHopper);
        Logger.recordOutput("Turret/RobotFieldPosition", robotFieldPosition);
        Logger.recordOutput("Turret/RobotPoseAngle", robotPoseAngle.getDegrees());
        Logger.recordOutput("Turret/robotToHopperFieldAngle", robotToHopperFieldAngle.getDegrees());
        Logger.recordOutput("Turret/robotPoseToHopperAngle", robotPoseToHopperAngle.getDegrees());
        // Constants.kField.;
        // turret.se(scaletarget.getDegrees());
        setTurretSetpoint(robotPoseToHopperAngle);
    }

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Alliance getAlliance() {
        System.out.println("returning alliance: " + DriverStation.getAlliance());
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    Translation2d UphomeR = new Translation2d(11.4, 5.5);
    Translation2d DownhomeR = new Translation2d(11.4, 2.5);
    Translation2d UphomeB = new Translation2d(4, 5.5);
    Translation2d DownhomeB = new Translation2d(4, 2.5);
    Translation2d target;

    public void getHomeDirection(boolean RedAlliance, double yPos) {
        if (RedAlliance) {
            if (yPos > 4) {
                target = UphomeR;
            } else {
                target = DownhomeR;
            }
        } else {
            if (yPos > 4) {
                target = UphomeB;
            } else {
                target = DownhomeB;
            }
        }
    }

    public void shootAtHome() {
        Translation2d robotFieldPosition = swerve.getPose().getTranslation();
        Rotation2d robotPoseAngle = swerve.getRotation();

        getHomeDirection(isRed(), robotFieldPosition.getY());

        var robotToTargetFieldAngle = target.minus(robotFieldPosition).getAngle();
        var robotPoseToTargetAngle = robotToTargetFieldAngle.minus(robotPoseAngle);
        setTurretSetpoint(robotPoseToTargetAngle);
    }

    public boolean isPastBack() {
        if ((turretMotor.getEncoder().getPosition() > 0.55
                        || turretMotor.getEncoder().getPosition() < -0.55)
                && Math.abs(Turretsetpoint) < 0.2) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            Turretsetpoint = turretMotor.getEncoder().getPosition();
            trapezoidSetpoint = new TrapezoidProfile.State(
                    turretMotor.getEncoder().getPosition(),
                    turretMotor.getEncoder().getVelocity());
            turretController.setSetpoint(trapezoidSetpoint.position, ControlType.kPosition);
        }

        trapezoidSetpoint =
                trapezoidProfile.calculate(0.02, trapezoidSetpoint, new TrapezoidProfile.State(Turretsetpoint, 0));
        turretController.setSetpoint(Turretsetpoint, ControlType.kPosition);
    }
}
